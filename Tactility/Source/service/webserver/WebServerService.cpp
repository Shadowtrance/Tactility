#ifdef ESP_PLATFORM

#include <Tactility/service/webserver/WebServerService.h>
#include <Tactility/service/webserver/AssetVersion.h>
#include <Tactility/service/ServiceManifest.h>
#include <Tactility/settings/WebServerSettings.h>
#include <Tactility/MountPoints.h>
#include <Tactility/file/File.h>
#include <Tactility/Log.h>
#include <Tactility/kernel/SystemEvents.h>
#include <Tactility/lvgl/Statusbar.h>

#include <esp_system.h>
#include <sstream>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <cctype>

namespace tt::service::webserver {

constexpr auto* TAG = "WebServer";

// Cached settings to avoid SD card reads on every HTTP request
static settings::webserver::WebServerSettings g_cachedSettings;
static bool g_settingsCached = false;

// Global instance pointer for controlling the service
static WebServerService* g_webServerInstance = nullptr;

// Core HTML - Minimal fail-safe interface hardcoded in firmware
constexpr auto* CORE_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Tactility WebServer</title>
    <style>
        body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; }
        h1 { color: #333; }
        .section { margin: 30px 0; padding: 20px; border: 1px solid #ddd; border-radius: 5px; }
        button { padding: 10px 20px; margin: 5px; cursor: pointer; }
        .primary { background-color: #007bff; color: white; border: none; }
        .danger { background-color: #dc3545; color: white; border: none; }
        input[type="file"] { margin: 10px 0; }
        a { color: #007bff; text-decoration: none; }
        a:hover { text-decoration: underline; }
    </style>
</head>
<body>
    <h1>Tactility Web Server</h1>
    <p><strong>Core System Interface</strong></p>
    
    <div class="section">
        <h2>Main Interface</h2>
        <p><a href="/dashboard.html">Open Dashboard →</a></p>
        <p><small>Access the full web interface with all features</small></p>
    </div>
    
    <div class="section">
        <h2>Asset Management</h2>
        <button class="primary" onclick="syncAssets()">Sync Web Assets</button>
        <p><small>Synchronize expanded HTML/CSS/JS between Data partition and SD card</small></p>
    </div>
    
    <div class="section">
        <h2>File Browser</h2>
        <p>Browse /data and /sdcard, upload or download files</p>
        <p><a href="/filebrowser">Open File Browser →</a></p>
    </div>
    
    <div class="section">
        <h2>System Actions</h2>
        <button class="danger" onclick="rebootDevice()">Reboot Device</button>
        <p><small>Restart the device</small></p>
    </div>
    
    <script>
        document.getElementById('uploadForm')?.addEventListener('submit', async (e) => {
            e.preventDefault();
            const formData = new FormData();
            const fileInput = document.getElementById('fileInput');
            formData.append('file', fileInput.files[0]);
            
            try {
                // Use consolidated /fs/upload endpoint (raw body expected)
                const response = await fetch('/fs/upload', {
                    method: 'POST',
                    body: formData.get('file')
                });
                const result = await response.text();
                document.getElementById('uploadStatus').textContent = 
                    response.ok ? 'Upload successful!' : 'Upload failed: ' + result;
            } catch (error) {
                document.getElementById('uploadStatus').textContent = 'Upload error: ' + error;
            }
        });
        
        async function syncAssets() {
            try {
                const response = await fetch('/admin/sync', { method: 'POST' });
                alert(response.ok ? 'Assets synchronized!' : 'Sync failed');
            } catch (error) {
                alert('Sync error: ' + error);
            }
        }
        
        async function rebootDevice() {
            if (confirm('Reboot device now?')) {
                try {
                    await fetch('/admin/reboot', { method: 'POST' });
                    alert('Device rebooting...');
                } catch (error) {
                    // Expected - device is rebooting
                }
            }
        }
    </script>
</body>
</html>
)HTML";

// Built-in file browser page (minimal, hardcoded)
constexpr auto* FILE_BROWSER_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Tactility File Browser</title>
        <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                #path { width: 60%; }
                table { border-collapse: collapse; width: 100%; margin-top: 10px; }
                th, td { border: 1px solid #ddd; padding: 6px; font-size: 14px; }
                th { background: #f0f0f0; }
                tr:hover { background: #f9f9f9; }
                button { padding: 6px 12px; margin: 4px; }
                .name { cursor: pointer; color: #007bff; }
                .name:hover { text-decoration: underline; }
                .toolbar { display:flex; gap:8px; flex-wrap:wrap; }
                input[type=file] { display:none; }
                .selected { background: #e6f7ff; }
                .disabled { opacity:0.4; pointer-events:none; }
        </style>
</head>
<body>
<h1>File Browser</h1>
<div class="toolbar">
    <button onclick="location.href='/'">Back</button>
    <input id="path" value="/data" />
    <button onclick="goRoot('/data')">/data</button>
    <button onclick="goRoot('/sdcard')">/sdcard</button>
    <button onclick="refresh()">Refresh</button>
    <button onclick="createFolder()">New Folder</button>
    <button onclick="deleteSelected()">Delete</button>
    <button onclick="renameSelected()">Rename</button>
    <button onclick="triggerUpload()">Upload</button>
    <input type="file" id="uploadInput" />
</div>
<p id="status"></p>
<table>
 <thead><tr><th>Name</th><th>Type</th><th>Size</th><th>Action</th></tr></thead>
 <tbody id="list"></tbody>
</table>
<script>
function api(path) { return `/fs/list?path=${encodeURIComponent(path)}`; }
function dl(path) { return `/fs/download?path=${encodeURIComponent(path)}`; }
function goRoot(root){ document.getElementById('path').value=root; refresh(); }
function goUp(){ const p=document.getElementById('path').value; if(p==='/'||p.split('/').filter(Boolean).length===1){return;} const parts=p.split('/'); parts.pop(); const np=parts.join('/')||'/'; document.getElementById('path').value=np; refresh(); }
function refresh(){ const p=document.getElementById('path').value; fetch(api(p)).then(r=>r.json()).then(d=>{ if(d.error){ setStatus(d.error,true); return;} render(d); setStatus(`Listed ${d.entries.length} item(s)`); updateDownloadState(); }).catch(e=>setStatus('Error: '+e,true)); }

let selectedName = '';
let selectedType = '';
function selectRow(name, tr, type) {
    document.querySelectorAll('#list tr').forEach(r=>r.classList.remove('selected'));
    if (name) { tr.classList.add('selected'); selectedName = name; selectedType = type; } else { selectedName = ''; selectedType = ''; }
    updateDownloadState();
}
function updateDownloadState(){ const btn=document.getElementById('downloadBtn'); if(!btn) return; if(selectedName && selectedType==='file') btn.classList.remove('disabled'); else btn.classList.add('disabled'); }

function render(d){ const tbody=document.getElementById('list'); tbody.innerHTML='';
    // add parent entry (..)
    if (d.path && d.path !== '/') {
        const up = document.createElement('tr');
        const nameTd = document.createElement('td');
        nameTd.className = 'name';
        nameTd.textContent = '..';
        nameTd.ondblclick = ()=>{ goUp(); };
        nameTd.onclick = (ev)=>{ selectRow('', up, 'dir'); };
        up.appendChild(nameTd);
        const typeTd = document.createElement('td'); typeTd.textContent='dir'; up.appendChild(typeTd);
        const sizeTd = document.createElement('td'); sizeTd.textContent=''; up.appendChild(sizeTd);
        const actTd = document.createElement('td'); up.appendChild(actTd);
        tbody.appendChild(up);
    }
    d.entries.forEach(e=>{ const tr=document.createElement('tr'); const nameTd=document.createElement('td'); nameTd.className='name'; nameTd.textContent=e.name; nameTd.ondblclick=()=>{ if(e.type==='dir'){ document.getElementById('path').value=(d.path==='/'? '' : d.path)+ '/' + e.name; refresh(); } else { window.location=dl((d.path==='/'? '' : d.path)+ '/' + e.name); } }; nameTd.onclick=(ev)=>{ selectRow(e.name, tr, e.type); }; tr.appendChild(nameTd); const typeTd=document.createElement('td'); typeTd.textContent=e.type; tr.appendChild(typeTd); const sizeTd=document.createElement('td'); sizeTd.textContent=e.type==='file'? e.size : ''; tr.appendChild(sizeTd); const actTd=document.createElement('td'); if(e.type==='file'){ const a=document.createElement('a'); a.href=dl((d.path==='/'? '' : d.path)+ '/' + e.name); a.textContent='Download'; actTd.appendChild(a);} tr.appendChild(actTd); tbody.appendChild(tr); }); }

function setStatus(msg,err){ const s=document.getElementById('status'); s.textContent=msg; s.style.color=err?'#c00':'#333'; }
function triggerUpload(){ document.getElementById('uploadInput').click(); }
document.getElementById('uploadInput').addEventListener('change', ev => { const file=ev.target.files[0]; if(!file){return;} const p=document.getElementById('path').value; const target=p+(p.endsWith('/')?'':'/')+file.name; fetch(`/fs/upload?path=${encodeURIComponent(target)}`, { method:'POST', body:file }).then(r=>r.text()).then(t=>{ setStatus(t); refresh(); }).catch(e=>setStatus('Upload error: '+e,true)); });

function downloadSelected(){ if(!selectedName){ setStatus('No file selected', true); return; } const p=document.getElementById('path').value; const url = dl((p==='/'? '' : p)+ '/' + selectedName); window.location = url; }

function createFolder(){ const name = prompt('New folder name:'); if(!name) return; const p=document.getElementById('path').value; const target = p+(p.endsWith('/')?'':'/')+name; fetch(`/fs/mkdir?path=${encodeURIComponent(target)}`, { method:'POST' }).then(r=>r.text()).then(t=>{ setStatus(t); refresh(); }).catch(e=>setStatus('mkdir error: '+e,true)); }

function deleteSelected(){ const p=document.getElementById('path').value; const name = selectedName; if(!name){ setStatus('No file/folder selected', true); return;} if(!confirm('Delete '+name+'?')) return; const target = p+(p.endsWith('/')?'':'/')+name; fetch(`/fs/delete?path=${encodeURIComponent(target)}`, { method:'POST' }).then(r=>r.text()).then(t=>{ setStatus(t); refresh(); }).catch(e=>setStatus('delete error: '+e,true)); }

function renameSelected(){ const p=document.getElementById('path').value; const name = selectedName; if(!name){ setStatus('No file/folder selected', true); return;} const newName = prompt('Rename to:', name); if(!newName) return; const target = p+(p.endsWith('/')?'':'/')+name; fetch(`/fs/rename?path=${encodeURIComponent(target)}&newName=${encodeURIComponent(newName)}`, { method:'POST' }).then(r=>r.text()).then(t=>{ setStatus(t); refresh(); }).catch(e=>setStatus('rename error: '+e,true)); }

refresh();
</script>
</body>
</html>
)HTML";

bool WebServerService::onStart(ServiceContext& service) {
    TT_LOG_I(TAG, "Starting WebServer service...");

    // Register global instance
    g_webServerInstance = this;

    // Create statusbar icon (hidden initially, shown when server actually starts)
    statusbarIconId = lvgl::statusbar_icon_add();
    lvgl::statusbar_icon_set_visibility(statusbarIconId, false);

    // Run asset synchronization on startup
    if (!syncAssets()) {
        TT_LOG_W(TAG, "Asset sync failed, but continuing with available assets");
    }

    // Load and cache settings once at boot
    g_cachedSettings = settings::webserver::loadOrGetDefault();
    g_settingsCached = true;
    TT_LOG_I(TAG, "HTTP server starting (access: %s)",
             g_cachedSettings.webServerEnabled ? "allowed" : "denied");

    // Subscribe to settings change events to refresh cache
    settingsEventSubscription = kernel::subscribeSystemEvent(
        kernel::SystemEvent::WebServerSettingsChanged,
        [](auto) {
            TT_LOG_I(TAG, "Settings changed, refreshing cache...");
            g_cachedSettings = settings::webserver::loadOrGetDefault();
            g_settingsCached = true;
        }
    );

    // Start HTTP server only if enabled in settings (default: OFF to save memory)
    if (g_cachedSettings.webServerEnabled) {
        TT_LOG_I(TAG, "WebServer enabled in settings, starting HTTP server...");
        setEnabled(true);
    } else {
        TT_LOG_I(TAG, "WebServer disabled in settings, NOT starting HTTP server (saves ~10KB RAM)");
        setEnabled(false);
    }

    return true;
}

void WebServerService::onStop(ServiceContext& service) {
    g_webServerInstance = nullptr;
    setEnabled(false);

    // Remove statusbar icon
    if (statusbarIconId >= 0) {
        lvgl::statusbar_icon_remove(statusbarIconId);
        statusbarIconId = -1;
    }
}

// region Enable/Disable

void WebServerService::setEnabled(bool enabled) {
    auto lock = mutex.asScopedLock();
    lock.lock();
    
    if (enabled) {
        if (!httpServer || !httpServer->isStarted()) {
            startServer();
        }
    } else {
        if (httpServer && httpServer->isStarted()) {
            stopServer();
        }
    }
}

bool WebServerService::isEnabled() const {
    auto lock = mutex.asScopedLock();
    lock.lock();
    return httpServer && httpServer->isStarted();
}

bool WebServerService::startServer() {
    auto settings = settings::webserver::loadOrGetDefault();
    
    // NOTE: If you see 'no slots left for registering handler', increase CONFIG_HTTPD_MAX_URI_HANDLERS in sdkconfig (default is 8, 16+ recommended for many endpoints)
    std::vector<httpd_uri_t> handlers = {
        {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = handleRoot,
            .user_ctx  = this
        },
        // Note: /upload removed in favor of POST /fs/upload handled by /fs/* dispatcher
        {
            .uri       = "/filebrowser",
            .method    = HTTP_GET,
            .handler   = handleFileBrowser,
            .user_ctx  = this
        },
        // Consolidated /fs/* handlers (dispatch internally) to save uri handler slots
        {
            .uri       = "/fs/*",
            .method    = HTTP_GET,
            .handler   = handleFsGenericGet,
            .user_ctx  = this
        },
        {
            .uri       = "/fs/*",
            .method    = HTTP_POST,
            .handler   = handleFsGenericPost,
            .user_ctx  = this
        },
        // Consolidated admin POST endpoints to save handler slots
        {
            .uri       = "/admin/*",
            .method    = HTTP_POST,
            .handler   = handleAdminPost,
            .user_ctx  = this
        },
        {
            .uri       = "/*",  // Catch-all for dynamic assets
            .method    = HTTP_GET,
            .handler   = handleAssets,
            .user_ctx  = this
        }
    };
    
    httpServer = std::make_unique<network::HttpServer>(
        settings.webServerPort,
        "0.0.0.0",
        handlers,
        8192  // Stack size
    );
    
    httpServer->start();
    if (!httpServer->isStarted()) {
        TT_LOG_E(TAG, "Failed to start HTTP server on port %d", settings.webServerPort);
        httpServer.reset();
        return false;
    }

    TT_LOG_I(TAG, "HTTP server started successfully on port %d", settings.webServerPort);
    kernel::publishSystemEvent(kernel::SystemEvent::WebServerStarted);

    // Show statusbar icon (different icon for AP vs Station mode)
    if (statusbarIconId >= 0) {
        const char* icon_name = (settings.wifiMode == settings::webserver::WiFiMode::AccessPoint)
            ? "webserver_ap_white.png"
            : "webserver_station_white.png";
        auto icon_path = std::string("A:/system/service/Statusbar/assets/") + icon_name;
        lvgl::statusbar_icon_set_image(statusbarIconId, icon_path);
        lvgl::statusbar_icon_set_visibility(statusbarIconId, true);
        TT_LOG_I(TAG, "WebServer statusbar icon shown (%s mode)",
                 settings.wifiMode == settings::webserver::WiFiMode::AccessPoint ? "AP" : "Station");
    }

    return true;
}

constexpr int SHUTDOWN_TIMEOUT_MS = 2000;
constexpr int SHUTDOWN_POLL_INTERVAL_MS = 100;
void WebServerService::stopServer() {
    if (httpServer) {
        // Request stop and wait briefly for graceful shutdown. Some underlying
        // HTTP server implementations may perform shutdown asynchronously,
        // so poll `isStarted()` for a short period before deciding it failed.
        httpServer->stop();

        const int maxWaitMs = SHUTDOWN_TIMEOUT_MS;
        const int stepMs = SHUTDOWN_POLL_INTERVAL_MS;
        int waited = 0;
        while (httpServer->isStarted() && waited < maxWaitMs) {
            vTaskDelay(pdMS_TO_TICKS(stepMs));
            waited += stepMs;
        }

        if (!httpServer->isStarted()) {
            httpServer.reset();
            TT_LOG_I(TAG, "HTTP server stopped");
            kernel::publishSystemEvent(kernel::SystemEvent::WebServerStopped);

            // Hide statusbar icon
            if (statusbarIconId >= 0) {
                lvgl::statusbar_icon_set_visibility(statusbarIconId, false);
                TT_LOG_I(TAG, "WebServer statusbar icon hidden");
            }
            return;
        }

        // Retry once if it didn't stop within the timeout
        TT_LOG_W(TAG, "HTTP server did not stop within %dms, retrying...", waited);
        httpServer->stop();
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!httpServer->isStarted()) {
            httpServer.reset();
            TT_LOG_I(TAG, "HTTP server stopped after retry");
            kernel::publishSystemEvent(kernel::SystemEvent::WebServerStopped);
            if (statusbarIconId >= 0) {
                lvgl::statusbar_icon_set_visibility(statusbarIconId, false);
                TT_LOG_I(TAG, "WebServer statusbar icon hidden");
            }
            return;
        }

        // If we still couldn't stop it, force cleanup to avoid leaving service
        // in a permanently-wedged state. Log error for diagnostics.
        TT_LOG_E(TAG, "HTTP server stop failed - server is still running after retries; forcing cleanup");
        httpServer.reset();
        kernel::publishSystemEvent(kernel::SystemEvent::WebServerStopped);
        if (statusbarIconId >= 0) {
            lvgl::statusbar_icon_set_visibility(statusbarIconId, false);
        }
    }
}

// region Endpoints



esp_err_t WebServerService::handleRoot(httpd_req_t* request) {
    
    TT_LOG_I(TAG, "GET /");
    
    if (httpd_resp_set_type(request, "text/html") != ESP_OK) {
        return ESP_FAIL;
    }
    
    if (httpd_resp_sendstr(request, CORE_HTML) != ESP_OK) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t WebServerService::handleUpload(httpd_req_t* request) {
    
    TT_LOG_I(TAG, "POST /upload");
    
    // TODO: Implement file upload handler
    // For now, just acknowledge
    httpd_resp_sendstr(request, "Upload endpoint - to be implemented");
    return ESP_OK;
}

// region File Browser helpers & handlers

static bool isAllowedBasePath(const std::string& path) {
    if (path.empty()) return false;
    // Check for ".." as a complete path component
    if (path == ".." || path.starts_with("../") || 
        path.find("/../") != std::string::npos || path.ends_with("/..")) {
        return false;
    }
    return path == "/data" || path.starts_with("/data/") || path == "/sdcard" || path.starts_with("/sdcard/");
}

// Normalize client-supplied path: URL-decode, trim quotes/control chars, ensure leading slash, collapse duplicate slashes
static std::string normalizePath(const std::string& raw) {
    // Helper: hex to int
    auto hexVal = [](char c)->int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        return -1;
    };

    std::string s = raw;
    // Remove surrounding single or double quotes
    if (s.size() >= 2 && ((s.front() == '\'' && s.back() == '\'') || (s.front() == '"' && s.back() == '"'))) {
        s = s.substr(1, s.size() - 2);
    }

    // URL-decode: %xx and '+' -> ' '
    std::string decoded;
    decoded.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (c == '%') {
            if (i + 2 < s.size()) {
                int hi = hexVal(s[i+1]);
                int lo = hexVal(s[i+2]);
                if (hi >= 0 && lo >= 0) {
                    decoded.push_back(static_cast<char>((hi << 4) | lo));
                    i += 2;
                    continue;
                }
            }
            // malformed %, keep it
            decoded.push_back(c);
        } else if (c == '+') {
            decoded.push_back(' ');
        } else {
            // strip control characters
            if (static_cast<unsigned char>(c) > 31) decoded.push_back(c);
        }
    }

    // Trim whitespace from ends
    size_t start = 0;
    while (start < decoded.size() && isspace((unsigned char)decoded[start])) ++start;
    size_t end = decoded.size();
    while (end > start && isspace((unsigned char)decoded[end-1])) --end;
    std::string trimmed = decoded.substr(start, end - start);

    // Ensure leading slash
    if (!trimmed.empty() && trimmed.front() != '/') trimmed = '/' + trimmed;
    if (trimmed.empty()) trimmed = "/";

    // Collapse duplicate slashes
    std::string out;
    out.reserve(trimmed.size());
    bool lastSlash = false;
    for (char c : trimmed) {
        if (c == '/') {
            if (!lastSlash) { out.push_back(c); lastSlash = true; }
        } else { out.push_back(c); lastSlash = false; }
    }

    return out;
}

static std::string escapeJson(const std::string& s) {
    std::ostringstream o;
    for (char c : s) {
        switch (c) {
            case '"': o << "\\\""; break;
            case '\\': o << "\\\\"; break;
            case '\n': o << "\\n"; break;
            case '\r': o << "\\r"; break;
            case '\t': o << "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)c;
                } else {
                    o << c;
                }
        }
    }
    return o.str();
}

static bool getQueryParam(httpd_req_t* req, const char* key, std::string& out) {
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1) return false;
    std::unique_ptr<char[]> buf(new char[len]);
    if (httpd_req_get_url_query_str(req, buf.get(), len) != ESP_OK) return false;
    char value[384];
    if (httpd_query_key_value(buf.get(), key, value, sizeof(value)) == ESP_OK) {
        out = value;
        return true;
    }
    return false;
}

esp_err_t WebServerService::handleFileBrowser(httpd_req_t* request) {
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_sendstr(request, FILE_BROWSER_HTML);
}

esp_err_t WebServerService::handleFsList(httpd_req_t* request) {
    std::string path;
    // Log raw query string for diagnostics
    size_t qlen = httpd_req_get_url_query_len(request) + 1;
    if (qlen > 1) {
        std::unique_ptr<char[]> qbuf(new char[qlen]);
        if (httpd_req_get_url_query_str(request, qbuf.get(), qlen) == ESP_OK) {
            TT_LOG_I(TAG, "GET /fs/list raw query: %s", qbuf.get());
        }
    }

    if (!getQueryParam(request, "path", path) || path.empty()) path = "/data";
    std::string norm = normalizePath(path);
    TT_LOG_I(TAG, "GET /fs/list decoded path: '%s' normalized: '%s'", path.c_str(), norm.c_str());
    if (!isAllowedBasePath(norm)) {
        TT_LOG_W(TAG, "GET /fs/list - invalid path requested: '%s' normalized: '%s'", path.c_str(), norm.c_str());
        httpd_resp_set_type(request, "application/json");
        httpd_resp_sendstr(request, "{\"error\":\"invalid path\"}");
        return ESP_OK;
    }
    std::vector<dirent> entries;
    std::ostringstream json;
    json << "{\"path\":\"" << norm << "\",\"entries\":[";
    int res = file::scandir(norm, entries, file::direntFilterDotEntries, nullptr);
    if (res < 0) {
        httpd_resp_set_type(request, "application/json");
        httpd_resp_sendstr(request, "{\"error\":\"scan failed\"}");
        return ESP_OK;
    }
    bool first = true;
    for (auto& e : entries) {
        if (!first) json << ','; else first = false;
        std::string name = e.d_name;
        bool is_dir = (e.d_type == file::TT_DT_DIR || e.d_type == file::TT_DT_CHR);
        std::string full = norm == "/" ? ("/" + name) : (norm + "/" + name);
        long size = 0;
        if (!is_dir) {
            FILE* fp = fopen(full.c_str(), "rb");
            if (fp) { fseek(fp, 0, SEEK_END); size = ftell(fp); fclose(fp);} }
        json << "{\"name\":\"" << escapeJson(name) << "\",\"type\":\"" << (is_dir?"dir":"file") << "\",\"size\":" << size << "}";
    }
    json << "]}";
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, json.str().c_str());
    return ESP_OK;
}

esp_err_t WebServerService::handleFsDownload(httpd_req_t* request) {
    std::string path;
    if (!getQueryParam(request, "path", path) || path.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "path required");
        return ESP_FAIL;
    }
    std::string norm = normalizePath(path);
    if (!isAllowedBasePath(norm) || !file::isFile(norm)) {
        TT_LOG_W(TAG, "GET /fs/download - not found or invalid path: '%s' normalized: '%s'", path.c_str(), norm.c_str());
        httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }
    // Content type guess
    const char* ct = "application/octet-stream";
    if (norm.ends_with(".txt")) ct = "text/plain";
    else if (norm.ends_with(".html")) ct = "text/html";
    else if (norm.ends_with(".json")) ct = "application/json";
    httpd_resp_set_type(request, ct);
    // Suggest download - build header into a local string so it remains valid
    std::string fname = file::getLastPathSegment(norm);
    std::string disposition = std::string("attachment; filename=\"") + fname + "\"";
    // RFC5987 fallback (filename*): percent-encode UTF-8 bytes for wider browser compatibility
    auto pctEncode = [](const std::string& s)->std::string{
        std::ostringstream oss;
        for (unsigned char c : s) {
            if (std::isalnum(c) || c=='-' || c=='.' || c=='_' || c=='~') {
                oss << c;
            } else {
                oss << '%';
                std::ostringstream hex;
                hex << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)c;
                oss << hex.str();
            }
        }
        return oss.str();
    };
    std::string pct = pctEncode(fname);
    if (!pct.empty()) {
        disposition += std::string("; filename*=UTF-8''") + pct;
    }
    // Set single Content-Disposition header (avoid adding duplicate headers)
    httpd_resp_set_hdr(request, "Content-Disposition", disposition.c_str());
    FILE* fp = fopen(norm.c_str(), "rb");
    if (!fp) { httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed"); return ESP_FAIL; }
    char buf[512]; size_t n;
    while ((n = fread(buf,1,sizeof(buf),fp))>0) {
        if (httpd_resp_send_chunk(request, buf, n) != ESP_OK) { fclose(fp); return ESP_FAIL; }
    }
    fclose(fp);
    httpd_resp_send_chunk(request, nullptr, 0);
    return ESP_OK;
}

esp_err_t WebServerService::handleFsUpload(httpd_req_t* request) {
    std::string path;

    // Log raw query and decoded path for diagnostics
    size_t qlen = httpd_req_get_url_query_len(request) + 1;
    if (qlen > 1) {
        std::unique_ptr<char[]> qbuf(new char[qlen]);
        if (httpd_req_get_url_query_str(request, qbuf.get(), qlen) == ESP_OK) {
            TT_LOG_I(TAG, "POST /fs/upload raw query: %s", qbuf.get());
        }
    }

    if (!getQueryParam(request, "path", path) || path.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "path required");
        return ESP_FAIL;
    }

    // Log decoded path and headers
    char content_type[64] = {0};
    httpd_req_get_hdr_value_str(request, "Content-Type", content_type, sizeof(content_type));
    std::string norm = normalizePath(path);
    TT_LOG_I(TAG, "POST /fs/upload decoded path: '%s' normalized: '%s' Content-Length: %d Content-Type: %s", path.c_str(), norm.c_str(), (int)request->content_len, content_type[0] ? content_type : "(null)");

    if (!isAllowedBasePath(norm)) {
        TT_LOG_W(TAG, "POST /fs/upload - invalid path requested: '%s' normalized: '%s'", path.c_str(), norm.c_str());
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "invalid path");
        return ESP_FAIL;
    }
    // Ensure parent directory exists
    file::findOrCreateParentDirectory(norm, 0755);
    FILE* fp = fopen(norm.c_str(), "wb");
    if (!fp) { httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed"); return ESP_FAIL; }
    char buf[512]; int remaining = request->content_len; int received=0;
    while (remaining > 0) {
        int to_read = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
        int ret = httpd_req_recv(request, buf, to_read);
        if (ret <= 0) { fclose(fp); httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed"); return ESP_FAIL; }
        size_t written = fwrite(buf, 1, ret, fp);
        if (written != (size_t)ret) {
            fclose(fp);
            httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
            return ESP_FAIL;
        }
        remaining -= ret;
        received += ret;
    }
    fclose(fp);
    httpd_resp_set_type(request, "text/plain");
    std::string msg = std::string("Uploaded ") + std::to_string(received) + " bytes";
    httpd_resp_sendstr(request, msg.c_str());
    return ESP_OK;
}

// Generic GET dispatcher for /fs/* URIs
esp_err_t WebServerService::handleFsGenericGet(httpd_req_t* request) {
    const char* uri = request->uri;
    if (strncmp(uri, "/fs/list", strlen("/fs/list")) == 0) return handleFsList(request);
    if (strncmp(uri, "/fs/download", strlen("/fs/download")) == 0) return handleFsDownload(request);
    if (strncmp(uri, "/fs/tree", strlen("/fs/tree")) == 0) return handleFsTree(request);
    TT_LOG_W(TAG, "GET %s - not found in fs generic dispatcher", uri);
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
    return ESP_FAIL;
}

// Generic POST dispatcher for /fs/* URIs
esp_err_t WebServerService::handleFsGenericPost(httpd_req_t* request) {
    const char* uri = request->uri;
    if (strncmp(uri, "/fs/mkdir", strlen("/fs/mkdir")) == 0) return handleFsMkdir(request);
    if (strncmp(uri, "/fs/delete", strlen("/fs/delete")) == 0) return handleFsDelete(request);
    if (strncmp(uri, "/fs/rename", strlen("/fs/rename")) == 0) return handleFsRename(request);
    if (strncmp(uri, "/fs/upload", strlen("/fs/upload")) == 0) return handleFsUpload(request);
    TT_LOG_W(TAG, "POST %s - not found in fs generic dispatcher", uri);
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
    return ESP_FAIL;
}

// Admin dispatcher for consolidated small POST endpoints (e.g. sync, reboot)
esp_err_t WebServerService::handleAdminPost(httpd_req_t* request) {
    const char* uri = request->uri;
    if (strncmp(uri, "/admin/sync", 11) == 0) return handleSync(request);
    if (strncmp(uri, "/admin/reboot", 13) == 0) return handleReboot(request);
    TT_LOG_W(TAG, "POST %s - not found in admin dispatcher", uri);
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
    return ESP_FAIL;
}

esp_err_t WebServerService::handleFsTree(httpd_req_t* request) {

    TT_LOG_I(TAG, "GET /fs/tree");

    std::ostringstream json;
    json << "{";
    // Gather mount points
    auto mounts = file::getMountPoints();
    json << "\"mounts\": [";
    bool firstMount = true;
    for (auto& m : mounts) {
        if (!firstMount) json << ','; else firstMount = false;
        std::string name = m.d_name;
        std::string path = (name == std::string("data") || name == std::string("/data")) ? std::string("/data") : std::string("/") + name;
        // normalize possible duplicate slash
        if (!path.starts_with("/")) path = std::string("/") + path;
        json << "{\"name\":\"" << name << "\",\"path\":\"" << path << "\",\"entries\": [";

        std::vector<dirent> entries;
        int res = file::scandir(path, entries, file::direntFilterDotEntries, nullptr);
        if (res > 0) {
            bool first = true;
            for (auto& e : entries) {
                if (!first) json << ','; else first = false;
                std::string en = e.d_name;
                bool is_dir = (e.d_type == file::TT_DT_DIR || e.d_type == file::TT_DT_CHR);
                json << "{\"name\":\"" << en << "\",\"type\":\"" << (is_dir?"dir":"file") << "\"}";
            }
        }

        json << "]}";
    }
    json << "]}";

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, json.str().c_str());
    return ESP_OK;
}

// Create a directory at the specified path (POST /fs/mkdir?path=/data/newdir)
esp_err_t WebServerService::handleFsMkdir(httpd_req_t* request) {
    std::string path;
    if (!getQueryParam(request, "path", path) || path.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "path required");
        return ESP_FAIL;
    }
    std::string norm = normalizePath(path);
    TT_LOG_I(TAG, "POST /fs/mkdir requested: '%s' normalized: '%s'", path.c_str(), norm.c_str());
    if (!isAllowedBasePath(norm)) {
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "invalid path");
        return ESP_FAIL;
    }
    bool ok = file::findOrCreateDirectory(norm, 0755);
    if (!ok) { httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "mkdir failed"); return ESP_FAIL; }
    httpd_resp_sendstr(request, "ok");
    return ESP_OK;
}

// Delete a file or directory (POST /fs/delete?path=/data/foo)
esp_err_t WebServerService::handleFsDelete(httpd_req_t* request) {
    std::string path;
    if (!getQueryParam(request, "path", path) || path.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "path required");
        return ESP_FAIL;
    }
    std::string norm = normalizePath(path);
    TT_LOG_I(TAG, "POST /fs/delete requested: '%s' normalized: '%s'", path.c_str(), norm.c_str());
    if (!isAllowedBasePath(norm)) {
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "invalid path");
        return ESP_FAIL;
    }
    bool ok = true;
    if (file::isDirectory(norm)) ok = file::deleteRecursively(norm);
    else if (file::isFile(norm)) ok = file::deleteFile(norm);
    else ok = false;
    if (!ok) { httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "delete failed"); return ESP_FAIL; }
    httpd_resp_sendstr(request, "ok");
    return ESP_OK;
}

// Rename a file or folder (POST /fs/rename?path=/data/oldname&newName=newname)
esp_err_t WebServerService::handleFsRename(httpd_req_t* request) {
    std::string path;
    std::string newName;
    if (!getQueryParam(request, "path", path) || path.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "path required");
        return ESP_FAIL;
    }
    if (!getQueryParam(request, "newName", newName) || newName.empty()) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "newName required");
        return ESP_FAIL;
    }
    std::string norm = normalizePath(path);
    TT_LOG_I(TAG, "POST /fs/rename requested: '%s' normalized: '%s' -> newName: '%s'", path.c_str(), norm.c_str(), newName.c_str());
    if (!isAllowedBasePath(norm)) {
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "invalid path");
        return ESP_FAIL;
    }

    // Basic validation of newName: must not contain path separators or '..'
    // Trim whitespace from newName
    auto trim = [](std::string& s){ size_t st=0; while (st<s.size() && isspace((unsigned char)s[st])) ++st; size_t ed=s.size(); while (ed>st && isspace((unsigned char)s[ed-1])) --ed; s = s.substr(st, ed-st); };
    trim(newName);
    if (newName.empty() || newName.find('/') != std::string::npos || newName.find('\\') != std::string::npos || newName.find("..") != std::string::npos) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid newName");
        return ESP_FAIL;
    }

    // compute parent directory
    std::string parent = "/";
    size_t pos = norm.find_last_of('/');
    if (pos != std::string::npos) {
        parent = (pos == 0) ? std::string("/") : norm.substr(0, pos);
    }

    if (!isAllowedBasePath(parent)) {
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "invalid target parent");
        return ESP_FAIL;
    }

    std::string target = file::getChildPath(parent, newName);

    // Prevent overwrite: fail if target exists
    if (file::isFile(target) || file::isDirectory(target)) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "target exists");
        return ESP_FAIL;
    }

    // perform rename
    int r = rename(norm.c_str(), target.c_str());
    if (r != 0) {
        int e = errno;
        TT_LOG_W(TAG, "rename failed errno=%d (%s) -> %s -> %s", e, strerror(e), norm.c_str(), target.c_str());
        // Return errno string to client to aid debugging
        std::string msg = std::string("rename failed: ") + strerror(e);
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, msg.c_str());
        return ESP_FAIL;
    }
    httpd_resp_sendstr(request, "ok");
    return ESP_OK;
}

// endregion

esp_err_t WebServerService::handleSync(httpd_req_t* request) {
    
    TT_LOG_I(TAG, "POST /sync");
    
    bool success = syncAssets();
    
    if (success) {
        httpd_resp_sendstr(request, "Assets synchronized successfully");
    } else {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Asset sync failed");
    }
    
    return success ? ESP_OK : ESP_FAIL;
}

esp_err_t WebServerService::handleReboot(httpd_req_t* request) {
    
    TT_LOG_I(TAG, "POST /reboot");
    
    httpd_resp_sendstr(request, "Rebooting...");
    
    // Reboot after a short delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

esp_err_t WebServerService::handleAssets(httpd_req_t* request) {
    
    const char* uri = request->uri;
    TT_LOG_I(TAG, "GET %s", uri);
    
    // Special case: if requesting dashboard.html but it doesn't exist, serve default.html
    std::string requestedPath = uri;
    std::string dataPath = std::string("/data/webserver") + requestedPath;
    
    if (strcmp(uri, "/dashboard.html") == 0 && !file::isFile(dataPath.c_str())) {
        // Dashboard doesn't exist, try default.html
        dataPath = "/data/webserver/default.html";
        TT_LOG_I(TAG, "dashboard.html not found, serving default.html");
    }
    
    // Try to serve from Data partition first
    if (file::isFile(dataPath.c_str())) {
        // Determine content type based on extension
        const char* contentType = "text/plain";
        if (strstr(uri, ".html")) contentType = "text/html";
        else if (strstr(uri, ".css")) contentType = "text/css";
        else if (strstr(uri, ".js")) contentType = "application/javascript";
        else if (strstr(uri, ".json")) contentType = "application/json";
        else if (strstr(uri, ".png")) contentType = "image/png";
        else if (strstr(uri, ".jpg") || strstr(uri, ".jpeg")) contentType = "image/jpeg";
        
        httpd_resp_set_type(request, contentType);
        
        // Read and send file using standard C FILE* operations
        auto lock = file::getLock(dataPath);
        lock->lock(portMAX_DELAY);
        
        FILE* fp = fopen(dataPath.c_str(), "rb");
        if (fp) {
            char buffer[512];
            size_t bytesRead;
            while ((bytesRead = fread(buffer, 1, sizeof(buffer), fp)) > 0) {
                if (httpd_resp_send_chunk(request, buffer, bytesRead) != ESP_OK) {
                    fclose(fp);
                    lock->unlock();
                    return ESP_FAIL;
                }
            }
            fclose(fp);
            lock->unlock();
            
            httpd_resp_send_chunk(request, nullptr, 0);  // End of chunks
            TT_LOG_I(TAG, "[200] %s (from Data)", uri);
            return ESP_OK;
        }
        lock->unlock();
    }
    
    // Fallback to SD card
    std::string sdPath = std::string("/sdcard/.tactility/webserver") + uri;
    if (file::isFile(sdPath.c_str())) {
        const char* contentType = "text/plain";
        if (strstr(uri, ".html")) contentType = "text/html";
        else if (strstr(uri, ".css")) contentType = "text/css";
        else if (strstr(uri, ".js")) contentType = "application/javascript";
        else if (strstr(uri, ".json")) contentType = "application/json";
        else if (strstr(uri, ".png")) contentType = "image/png";
        else if (strstr(uri, ".jpg") || strstr(uri, ".jpeg")) contentType = "image/jpeg";
        
        httpd_resp_set_type(request, contentType);
        
        auto lock = file::getLock(sdPath);
        lock->lock(portMAX_DELAY);
        
        FILE* fp = fopen(sdPath.c_str(), "rb");
        if (fp) {
            char buffer[512];
            size_t bytesRead;
            while ((bytesRead = fread(buffer, 1, sizeof(buffer), fp)) > 0) {
                if (httpd_resp_send_chunk(request, buffer, bytesRead) != ESP_OK) {
                    fclose(fp);
                    lock->unlock();
                    return ESP_FAIL;
                }
            }
            fclose(fp);
            lock->unlock();
            
            httpd_resp_send_chunk(request, nullptr, 0);  // End of chunks
            TT_LOG_I(TAG, "[200] %s (from SD)", uri);
            return ESP_OK;
        }
        lock->unlock();
    }
    
    // File not found
    TT_LOG_W(TAG, "[404] %s", uri);
    httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "File not found");
    return ESP_FAIL;
}

extern const ServiceManifest manifest = {
    .id = "WebServer",
    .createService = create<WebServerService>
};

void setWebServerEnabled(bool enabled) {
    if (g_webServerInstance != nullptr) {
        g_webServerInstance->setEnabled(enabled);
        // Don't log here - startServer()/stopServer() already log the actual result
    } else {
        TT_LOG_W(TAG, "WebServer service not available, cannot %s",
                 enabled ? "start" : "stop");
    }
}

} // namespace

#endif
