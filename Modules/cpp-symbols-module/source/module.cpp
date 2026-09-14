// SPDX-License-Identifier: Apache-2.0
#include <cpp_symbols/module.h>

#include <cstddef>
#include <cstdint>
#include <new>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if defined(__GLIBCXX__) || defined(ESP_PLATFORM)
#define TT_CPP_SYMBOLS_AVAILABLE 1
#include <bits/functexcept.h>
#else
#define TT_CPP_SYMBOLS_AVAILABLE 0
#endif

#if TT_CPP_SYMBOLS_AVAILABLE
extern "C" {
    // cplusplus: compiler/runtime ABI support
#ifdef ESP_PLATFORM
    // Mangled for a 32-bit ABI ("j" = unsigned int, i.e. size_t on ESP32's ILP32). A 64-bit host's
    // libstdc++ exports these under different (m-suffixed) mangled names, so they don't apply there.
    extern void* _Znwj(uint32_t size); // operator new(unsigned int)
    extern void _ZdlPvj(void* p, uint64_t size); // operator delete(void*, unsigned int)
    extern void* _Znaj(uint32_t size); // operator new[](unsigned int)
    extern void _ZdaPvj(void* p, uint64_t size); // operator delete[](void*, unsigned int)
    // Unsized forms: the compiler picks these over the sized ones above depending on context
    // (e.g. trivially-destructible types needing no array cookie), so both must be exported.
    extern void _ZdlPv(void* p); // operator delete(void*)
    extern void _ZdaPv(void* p); // operator delete[](void*)
#endif
    extern void __cxa_pure_virtual();
    // cxx_guards.cpp
    extern int __cxa_guard_acquire(void* pg);
    extern void __cxa_guard_release(void* pg) throw();
    extern void __cxa_guard_abort(void* pg) throw();
#ifdef ESP_PLATFORM
    // Not part of the Itanium C++ ABI that desktop libstdc++ implements; ESP-IDF's toolchain only.
    extern void __cxa_guard_dummy(void);
#endif

    // stl: std::map / std::set red-black tree non-template helpers. We use the mangled names
    // directly (same pattern as the basic_string cold path below) to avoid ambiguity from the
    // overloaded const/non-const variants in stl_tree.h.
    void* _ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base(void*);
    void* _ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base(void*);
    void  _ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_(bool, void*, void*, void*);

#ifdef ESP_PLATFORM
    // string - same 32-bit-ABI mangling caveat as operator new/delete above.
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE15_M_replace_coldEPcjPKcjj(void*, char*, unsigned int, char const*, unsigned int, unsigned int);
    void* _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6substrEjj(void*, const void*, unsigned int, unsigned int);
    char* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERjj(void*, unsigned int*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEj(void*, unsigned int);
    unsigned int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEcj(const void*, char, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_disposeEv(void*);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEjjPKcj(void*, unsigned int, unsigned int, const char*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE12_M_constructIPKcEEvT_S8_St20forward_iterator_tag(void*, const char*, const char*, char);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6appendEPKc(void*, const char*);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6appendEPKcj(void*, const char*, unsigned int);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6assignEPKc(void*, const char*);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7_S_copyEPcPKcj(char*, const char*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7_S_moveEPcPKcj(char*, const char*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE8_M_eraseEjj(void*, unsigned int, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE8pop_backEv(void*);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcj(void*, const char*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_(void*, const void*);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_mutateEjjPKcj(void*, unsigned int, unsigned int, const char*, unsigned int);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9push_backEc(void*, char);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_(void*, void*);
    // Non-members: return basic_string<char> by value, so the first param is the hidden
    // return-value pointer (Itanium ABI), same convention as substr() above.
    void* _ZSt12__str_concatINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEET_PKNS6_10value_typeENS6_9size_typeES9_SA_RKNS6_14allocator_typeE(void*, const char*, unsigned int, const char*, unsigned int, const void*);
    void* _ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EERKS8_PKS5_(void*, const void*, const char*);
    // More basic_string members needed by AudiobookPlayer (path/filename manipulation).
    unsigned int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEPKcj(const void*, const char*, unsigned int);
    unsigned int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEPKcjj(const void*, const char*, unsigned int, unsigned int);
    unsigned int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE5rfindEPKcjj(const void*, const char*, unsigned int, unsigned int);
    unsigned int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE5rfindEcj(const void*, char, unsigned int);
    int _ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7compareERKS4_(const void*, const void*);
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_(void*, void*);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6insertEjPKc(void*, unsigned int, const char*);
    void* _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7replaceEjjPKcj(void*, unsigned int, unsigned int, const char*, unsigned int);
    unsigned int _ZNSt8__detail14__to_chars_lenIjEEjT_i(unsigned int, int);
    void _ZNSt8__detail18__to_chars_10_implIjEEvPcjT_(char*, unsigned int, unsigned int);
    bool _ZSteqIcSt11char_traitsIcESaIcEEbRKNSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_(const void*, const char*); // operator==(string const&, const char*)
    bool _ZSteqIcSt11char_traitsIcESaIcEEbRKNSt7__cxx1112basic_stringIT_T0_T1_EESA_(const void*, const void*); // operator==(string const&, string const&)
    // operator+ overloads: hidden return-value pointer (return basic_string by value).
    void* _ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEOS8_S9_(void*, void*, void*);
    void* _ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_(void*, const char*, const void*);
    void* _ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EERKS8_SA_(void*, const void*, const void*);
    // vector<unsigned char> and vector<std::string> internals - Note: mangled names required.
    void _ZNKSt6vectorIhSaIhEE12_M_check_lenEjPKc(const void*, unsigned int, const char*);
    void _ZNKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE12_M_check_lenEjPKc(const void*, unsigned int, const char*);
    bool _ZNKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE5emptyEv(const void*);
    void* _ZNSt12_Vector_baseIhSaIhEE11_M_allocateEj(void*, unsigned int);
    void* _ZNSt12_Vector_baseIhSaIhEE17_M_create_storageEj(void*, unsigned int);
    void _ZNSt6vectorIhSaIhEE17_M_default_appendEj(void*, unsigned int);
    void _ZNSt6vectorIhSaIhEE6resizeEj(void*, unsigned int);
    unsigned char* _ZNSt27__uninitialized_default_n_1ILb1EE18__uninit_default_nIPhjEET_S3_T0_(unsigned char*, unsigned int);
    void* _ZSt9__fill_a1IhhEN9__gnu_cxx11__enable_ifIXaasrSt9__is_byteIT_E7__valueoosrSt10__are_sameIS3_T0_E7__valuesrSt20__memcpyable_integerIS6_E7__widthEvE6__typeEPS3_SC_RKS6_(unsigned char*, unsigned char*, const unsigned char*);
    // std::mutex
    void _ZNSt5mutex4lockEv(void*);
    // Remaining basic_string/vector template instantiations, raw-extern like _M_construct's
    // forward-iterator overload above - these are genuinely out-of-line template
    // instantiations (not compiler-inlined ctors/dtors), so no forcing wrapper is needed.
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE12_M_constructILb1EEEvPKcj(void*, const char*, unsigned int);
    // resize_and_overwrite's instantiation is scoped to to_string(unsigned)'s own private lambda
    // type - unnameable directly, but calling std::to_string(unsigned) below (construct_
    // to_string_result) instantiates it in this TU under this exact mangled name, addressable
    // by name even though its C++ type can't be spelled outside to_string's own body.
    void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE20resize_and_overwriteIRZNS_9to_stringEjEUlPcjE_EEvjT_(void*, unsigned int, void*);
    void* _ZSt14__relocate_a_1IPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_SaIS5_EET0_T_S9_S8_RT1_(void*, void*, void*, void*);
    int _ZStssIcSt11char_traitsIcESaIcEEDTcl21__char_traits_cmp_catIT0_ELi0EEERKNSt7__cxx1112basic_stringIT_S3_T1_EESB_(const void*, const void*); // operator<=>(string const&, string const&)
#endif
}

#ifdef ESP_PLATFORM
namespace {
// basic_string(basic_string const&, pos, len) has no out-of-line definition anywhere to take the
// address of - unlike everything else above, GCC never emits one for this ctor under C++20+ (it's
// a pure header-inline forwarder around _M_construct(), confirmed by trying to force an
// instantiation and finding no resulting linkable symbol). Implemented directly instead: this
// function's placement-new triggers the compiler to inline the real construction logic here,
// producing a genuine addressable definition, registered below under the mangled name(s) the ELF
// loader actually looks up.
void construct_basic_string_from_substring(void* self, const void* str, unsigned int pos, unsigned int len) {
    new (self) std::string(*static_cast<const std::string*>(str), pos, len);
}
// Same story for basic_string(const char*, allocator<char> const&).
void construct_basic_string_from_cstr(void* self, const char* s, const void* alloc) {
    new (self) std::string(s, *static_cast<const std::allocator<char>*>(alloc));
}
// Same story for the move constructor, basic_string(basic_string&&).
void construct_basic_string_move(void* self, void* other) {
    new (self) std::string(std::move(*static_cast<std::string*>(other)));
}

// vector<std::string>: same story as basic_string's ctors above - the destructor for a
// non-trivial element type has no out-of-line definition anywhere to take the address of, so a
// wrapper that actually destroys one forces the compiler to emit a genuine, addressable
// definition here, registered below under the mangled name the ELF loader looks up.
void destroy_vector_of_strings(void* self) {
    static_cast<std::vector<std::string>*>(self)->~vector();
}
// Same story for __new_allocator<std::string>::allocate() (used internally by vector<string>'s
// growth/reserve path).
void* allocate_string_storage(void* self, unsigned int n, const void* hint) {
    return static_cast<std::__new_allocator<std::string>*>(self)->allocate(n, hint);
}

// Same story for vector<unsigned char>'s destructor.
void destroy_vector_of_bytes(void* self) {
    static_cast<std::vector<unsigned char>*>(self)->~vector();
}

// _Vector_base<T>::~_Vector_base only deallocates the raw buffer (element destruction is
// vector<T>::~vector()'s job, called before this) - a distinct entry point with no accessible
// out-of-line definition of its own (it's a protected base of vector<T>, and its destructor body
// is a one-line inline in the header). A shim that publicly re-derives from it regains access to
// call the real destructor at the right address, without duplicating its (private) cleanup logic.
struct StringVectorBaseShim : std::_Vector_base<std::string, std::allocator<std::string>> {};
void destroy_vector_base_of_strings(void* self) {
    static_cast<StringVectorBaseShim*>(self)->~StringVectorBaseShim();
}

// std::to_string(unsigned) is `inline` in the header (no prebuilt out-of-line definition in
// libstdc++'s archive) - the app calls it directly as an opaque function in at least one call
// site (rather than always inlining it), so it needs a real out-of-line definition under its own
// mangled name, not just a forcing side effect. Hidden return-value pointer convention, matching
// the other string-returning functions (operator+, __str_concat) elsewhere in this file. This
// same call also instantiates resize_and_overwrite()'s to_string-private lambda, registered
// separately below under its own mangled name.
void construct_to_string_result(void* out, unsigned int value) {
    new (out) std::string(std::to_string(value));
}

// basic_string::rfind<string_view>(string_view const&, pos) is a small header-inline SFINAE
// forwarder around the already-exported rfind(const char*, pos, n) overload - no prebuilt
// out-of-line definition either, same story as to_string above.
[[gnu::used]] unsigned int rfind_string_view(const std::string& self, const std::string_view& sv, unsigned int pos) {
    return self.rfind(sv, pos);
}

// _Vector_base<T>::_Vector_impl_data::_M_swap_data - a plain one-line inline swap of three
// pointers, normally never emitted out-of-line, but the app references it directly anyway
// (matches _M_replace_cold's "kept out of line" pattern elsewhere in this file). Its type and
// method are public members of the (struct, so default-public) _Vector_base.
using StringVectorImplData = std::_Vector_base<std::string, std::allocator<std::string>>::_Vector_impl_data;
void swap_string_vector_impl_data(void* self, void* other) {
    static_cast<StringVectorImplData*>(self)->_M_swap_data(*static_cast<StringVectorImplData*>(other));
}
// Same story for vector<unsigned char>(size_type, allocator const&).
void construct_vector_of_bytes_sized(void* self, unsigned int count, const void* alloc) {
    new (self) std::vector<unsigned char>(count, *static_cast<const std::allocator<unsigned char>*>(alloc));
}

// Same _Vector_base<T>::~_Vector_base distinction as the string version above, for
// vector<unsigned char>.
struct ByteVectorBaseShim : std::_Vector_base<unsigned char, std::allocator<unsigned char>> {};
void destroy_vector_base_of_bytes(void* self) {
    static_cast<ByteVectorBaseShim*>(self)->~ByteVectorBaseShim();
}

// push_back/emplace_back/back on vector<string> are ordinary public member functions with
// out-of-line definitions, registered below under their real mangled names - the app's own
// push_back/emplace_back calls resolve directly here, so each must insert exactly once.
void push_back_string(void* self, const std::string& value) {
    static_cast<std::vector<std::string>*>(self)->push_back(value);
}
std::string& emplace_back_string(void* self, std::string&& value) {
    return static_cast<std::vector<std::string>*>(self)->emplace_back(std::move(value));
}

// _M_realloc_append(T&&) is what push_back/emplace_back call internally when the vector needs
// to grow - the app can also call it directly (its own compiled code inlined the fast/slow-path
// split from push_back's body, keeping only the growth call out-of-line). These do exactly what
// _M_realloc_append itself does (append one element, growing the vector), so redirecting the
// app's call here is behavior-preserving - unlike push_back_string/emplace_back_string above,
// which append TWO elements and must not be reused for this.
void realloc_append_string_const_ref(void* self, const std::string& value) {
    static_cast<std::vector<std::string>*>(self)->push_back(value);
}
void realloc_append_string_rvalue(void* self, std::string&& value) {
    static_cast<std::vector<std::string>*>(self)->push_back(std::move(value));
}

// _Guard_alloc is a private RAII rollback guard nested inside vector<T>::_M_realloc_append's own
// body (rolls back the new buffer if appending throws mid-copy) - not derivable-into like
// _Vector_base above, since it's private to vector<T> itself rather than a protected base. Its
// layout is fixed by the header ({pointer storage, size_type len, _Base& vect}) and its destructor
// body is just "deallocate storage via the vector's allocator if non-null", so this replicates
// that logic directly against the same layout instead of calling the inaccessible real dtor.
template <typename T>
struct GuardAllocLayout {
    T* storage;
    std::size_t len;
    void* vect;
};
template <typename T>
void destroy_guard_alloc(void* self) {
    auto* guard = static_cast<GuardAllocLayout<T>*>(self);
    if (guard->storage) {
        std::allocator<T>().deallocate(guard->storage, guard->len);
    }
}
std::string& back_of_string_vector(void* self) {
    return static_cast<std::vector<std::string>*>(self)->back();
}
// vector<string>'s move-assignment operator (self = std::move(other)) - forces the private
// _M_move_assign/_M_swap_data helpers it needs to be emitted as addressable definitions too.
void move_assign_string_vector(void* self, void* other) {
    *static_cast<std::vector<std::string>*>(self) = std::move(*static_cast<std::vector<std::string>*>(other));
}
}
#endif
#endif

static const ModuleSymbol SYMBOLS[] = {
#if TT_CPP_SYMBOLS_AVAILABLE
    // cplusplus
#ifdef ESP_PLATFORM
    DEFINE_MODULE_SYMBOL(_Znwj), // operator new(unsigned int)
    DEFINE_MODULE_SYMBOL(_ZdlPvj), // operator delete(void*, unsigned int)
    DEFINE_MODULE_SYMBOL(_Znaj), // operator new[](unsigned int)
    DEFINE_MODULE_SYMBOL(_ZdaPvj), // operator delete[](void*, unsigned int)
    DEFINE_MODULE_SYMBOL(_ZdlPv), // operator delete(void*)
    DEFINE_MODULE_SYMBOL(_ZdaPv), // operator delete[](void*)
#endif
    { "_ZSt7nothrow", (void*)&std::nothrow },
    DEFINE_MODULE_SYMBOL(__cxa_pure_virtual), // class-related, see https://arobenko.github.io/bare_metal_cpp/
    DEFINE_MODULE_SYMBOL(__cxa_guard_acquire),
    DEFINE_MODULE_SYMBOL(__cxa_guard_release),
    DEFINE_MODULE_SYMBOL(__cxa_guard_abort),
#ifdef ESP_PLATFORM
    DEFINE_MODULE_SYMBOL(__cxa_guard_dummy),
#endif
    // stl - Note: You have to use the mangled names here
    { "_ZSt17__throw_bad_allocv", (void*)&(std::__throw_bad_alloc) },
    { "_ZSt28__throw_bad_array_new_lengthv", (void*)&(std::__throw_bad_array_new_length) },
    { "_ZSt25__throw_bad_function_callv", (void*)&(std::__throw_bad_function_call) },
    { "_ZSt20__throw_length_errorPKc", (void*)&(std::__throw_length_error) },
    { "_ZSt19__throw_logic_errorPKc", (void*)&std::__throw_logic_error },
    { "_ZSt24__throw_out_of_range_fmtPKcz", (void*)&std::__throw_out_of_range_fmt },
    { "_ZSt20__throw_system_errori", (void*)&std::__throw_system_error },
    // stl - std::map / std::set (red-black tree internals)
    DEFINE_MODULE_SYMBOL(_ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base),
    DEFINE_MODULE_SYMBOL(_ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base),
    DEFINE_MODULE_SYMBOL(_ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_),
#ifdef ESP_PLATFORM
    // string - Note: You have to use the mangled names here
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE15_M_replace_coldEPcjPKcjj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6substrEjj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERjj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_disposeEv),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEjjPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE12_M_constructIPKcEEvT_S8_St20forward_iterator_tag),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6appendEPKc),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6appendEPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6assignEPKc),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7_S_copyEPcPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7_S_moveEPcPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE8_M_eraseEjj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE8pop_backEv),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_mutateEjjPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9push_backEc),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_),
    // C1/C2/C5: complete-object, base-object, and comdat-folded aliases of the same constructor.
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC1ERKS4_jj", (void*)&construct_basic_string_from_substring },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2ERKS4_jj", (void*)&construct_basic_string_from_substring },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC5ERKS4_jj", (void*)&construct_basic_string_from_substring },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC1IS3_EEPKcRKS3_", (void*)&construct_basic_string_from_cstr },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_", (void*)&construct_basic_string_from_cstr },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC5IS3_EEPKcRKS3_", (void*)&construct_basic_string_from_cstr },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC1EOS4_", (void*)&construct_basic_string_move },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2EOS4_", (void*)&construct_basic_string_move },
    { "_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC5EOS4_", (void*)&construct_basic_string_move },
    DEFINE_MODULE_SYMBOL(_ZSt12__str_concatINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEET_PKNS6_10value_typeENS6_9size_typeES9_SA_RKNS6_14allocator_typeE),
    DEFINE_MODULE_SYMBOL(_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EERKS8_PKS5_),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEPKcj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4findEPKcjj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE5rfindEPKcjj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE5rfindEcj),
    DEFINE_MODULE_SYMBOL(_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7compareERKS4_),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE6insertEjPKc),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7replaceEjjPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt8__detail14__to_chars_lenIjEEjT_i),
    DEFINE_MODULE_SYMBOL(_ZNSt8__detail18__to_chars_10_implIjEEvPcjT_),
    DEFINE_MODULE_SYMBOL(_ZSteqIcSt11char_traitsIcESaIcEEbRKNSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_),
    DEFINE_MODULE_SYMBOL(_ZSteqIcSt11char_traitsIcESaIcEEbRKNSt7__cxx1112basic_stringIT_T0_T1_EESA_),
    DEFINE_MODULE_SYMBOL(_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEOS8_S9_),
    DEFINE_MODULE_SYMBOL(_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_),
    DEFINE_MODULE_SYMBOL(_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EERKS8_SA_),
    DEFINE_MODULE_SYMBOL(_ZNKSt6vectorIhSaIhEE12_M_check_lenEjPKc),
    DEFINE_MODULE_SYMBOL(_ZNKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE12_M_check_lenEjPKc),
    DEFINE_MODULE_SYMBOL(_ZNKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE5emptyEv),
    DEFINE_MODULE_SYMBOL(_ZNSt12_Vector_baseIhSaIhEE11_M_allocateEj),
    DEFINE_MODULE_SYMBOL(_ZNSt12_Vector_baseIhSaIhEE17_M_create_storageEj),
    DEFINE_MODULE_SYMBOL(_ZNSt6vectorIhSaIhEE17_M_default_appendEj),
    DEFINE_MODULE_SYMBOL(_ZNSt6vectorIhSaIhEE6resizeEj),
    DEFINE_MODULE_SYMBOL(_ZNSt27__uninitialized_default_n_1ILb1EE18__uninit_default_nIPhjEET_S3_T0_),
    DEFINE_MODULE_SYMBOL(_ZSt9__fill_a1IhhEN9__gnu_cxx11__enable_ifIXaasrSt9__is_byteIT_E7__valueoosrSt10__are_sameIS3_T0_E7__valuesrSt20__memcpyable_integerIS6_E7__widthEvE6__typeEPS3_SC_RKS6_),
    DEFINE_MODULE_SYMBOL(_ZNSt5mutex4lockEv),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE12_M_constructILb1EEEvPKcj),
    DEFINE_MODULE_SYMBOL(_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE20resize_and_overwriteIRZNS_9to_stringEjEUlPcjE_EEvjT_),
    { "_ZNKSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE5rfindISt17basic_string_viewIcS2_EEENSt9enable_ifIXsrSt6__and_IJSt14is_convertibleIRKT_S7_ESt6__not_ISA_IPSC_PKS4_EESF_ISA_ISD_PKcEEEE5valueEjE4typeESD_j", (void*)&rfind_string_view },
    DEFINE_MODULE_SYMBOL(_ZSt14__relocate_a_1IPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_SaIS5_EET0_T_S9_S8_RT1_),
    DEFINE_MODULE_SYMBOL(_ZStssIcSt11char_traitsIcESaIcEEDTcl21__char_traits_cmp_catIT0_ELi0EEERKNSt7__cxx1112basic_stringIT_S3_T1_EESB_),
    // vector<std::string> - Note: You have to use the mangled names here
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev", (void*)&destroy_vector_of_strings },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED2Ev", (void*)&destroy_vector_of_strings },
    { "_ZNSt15__new_allocatorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE8allocateEjPKv", (void*)&allocate_string_storage },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE9push_backERKS5_", (void*)&push_back_string },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE12emplace_backIJS5_EEERS5_DpOT_", (void*)&emplace_back_string },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE4backEv", (void*)&back_of_string_vector },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE14_M_move_assignEOS7_St17integral_constantIbLb1EE", (void*)&move_assign_string_vector },
    // vector<unsigned char> - Note: You have to use the mangled names here
    { "_ZNSt6vectorIhSaIhEED1Ev", (void*)&destroy_vector_of_bytes },
    { "_ZNSt6vectorIhSaIhEED2Ev", (void*)&destroy_vector_of_bytes },
    { "_ZNSt6vectorIhSaIhEEC1EjRKS0_", (void*)&construct_vector_of_bytes_sized },
    { "_ZNSt12_Vector_baseIhSaIhEED2Ev", (void*)&destroy_vector_base_of_bytes },
    // _Vector_base<T>::~_Vector_base only deallocates the raw buffer (elements are destroyed by
    // vector<T>'s own destructor before this runs) - a distinct entry point from ~vector(), so
    // it needs its own wrapper rather than reusing destroy_vector_of_strings/_of_bytes above.
    { "_ZNSt12_Vector_baseINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED2Ev", (void*)&destroy_vector_base_of_strings },
    { "_ZNSt7__cxx119to_stringEj", (void*)&construct_to_string_result },
    { "_ZNSt12_Vector_baseINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_Vector_impl_data12_M_swap_dataERS8_", (void*)&swap_string_vector_impl_data },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_appendIJRKS5_EEEvDpOT_", (void*)&realloc_append_string_const_ref },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_appendIJS5_EEEvDpOT_", (void*)&realloc_append_string_rvalue },
    { "_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE12_Guard_allocD1Ev", (void*)&destroy_guard_alloc<std::string> },
    { "_ZNSt6vectorIhSaIhEE12_Guard_allocD1Ev", (void*)&destroy_guard_alloc<unsigned char> },
#endif
#endif // TT_CPP_SYMBOLS_AVAILABLE
    MODULE_SYMBOL_TERMINATOR
};

extern "C" {

Module cpp_symbols_module = {
    .name = "cpp-symbols",
    .start = nullptr,
    .stop = nullptr,
    .drivers = nullptr,
    .symbols = SYMBOLS,
    .internal = nullptr,
};

}
