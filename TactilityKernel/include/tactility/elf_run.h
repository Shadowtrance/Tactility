// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <tactility/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Loads an ELF image from memory, runs its entry point to completion, and unloads it.
 *
 * This is the mechanism the OS itself uses to launch apps, exposed so that a loaded app can in turn
 * run further ELF images of its own - a shell running its own command binaries, for example.
 *
 * Loading is flat rather than nested: relocations in @p elf_data are resolved against the firmware's
 * exported symbol table, exactly as they are for an app launched by the OS. An image run from inside
 * another app therefore sees the same symbols as one launched from the launcher, and specifically
 * does NOT see symbols belonging to the app that called this function.
 *
 * The entry point runs on the calling task, so that task's stack must be large enough for whatever
 * the image does. Call this from a dedicated task if the caller's own stack is modest.
 *
 * @warning Not reentrant with respect to a single image: @p elf_data must stay valid and unmodified
 * for the whole call.
 *
 * @param[in] elf_data the complete ELF image
 * @param[in] argc argument count passed to the image's entry point
 * @param[in] argv argument vector passed to the image's entry point
 * @param[out] exit_code the value the entry point returned, or NULL to ignore it
 * @retval ERROR_NONE when the image loaded, ran, and unloaded
 * @retval ERROR_INVALID_ARGUMENT when elf_data is NULL
 * @retval ERROR_OUT_OF_MEMORY when the image could not be allocated
 * @retval ERROR_RESOURCE when the image could not be relocated (e.g. it needs a symbol the firmware
 * does not export) or failed to start
 */
error_t elf_run(const uint8_t* elf_data, int argc, char* argv[], int* exit_code);

#ifdef __cplusplus
}
#endif
