// SPDX-License-Identifier: Apache-2.0
#include <tactility/elf_run.h>
#include <tactility/log.h>

#include <esp_elf.h>

static const char* TAG = "ElfRun";

error_t elf_run(const uint8_t* elf_data, int argc, char* argv[], int* exit_code) {
    if (elf_data == NULL) {
        return ERROR_INVALID_ARGUMENT;
    }

    esp_elf_t elf;
    if (esp_elf_init(&elf) != 0) {
        LOG_E(TAG, "Failed to initialize ELF object");
        return ERROR_OUT_OF_MEMORY;
    }

    // Relocation resolves against the firmware's exported symbol table. A negative result is a
    // negated errno value - most commonly -ENOSYS when the image needs a symbol we don't export.
    const int relocate_result = esp_elf_relocate(&elf, elf_data);
    if (relocate_result != 0) {
        LOG_E(TAG, "Failed to relocate ELF: %d", relocate_result);
        esp_elf_deinit(&elf);
        return ERROR_RESOURCE;
    }

    if (elf.entry == NULL) {
        LOG_E(TAG, "ELF has no entry point");
        esp_elf_deinit(&elf);
        return ERROR_RESOURCE;
    }

    // The entry point is called directly rather than through esp_elf_request(), which discards the
    // return value - a shell running command binaries needs their exit status.
    const int result = elf.entry(argc, argv);
    if (exit_code != NULL) {
        *exit_code = result;
    }

    esp_elf_deinit(&elf);
    return ERROR_NONE;
}
