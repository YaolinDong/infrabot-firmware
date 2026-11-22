# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Refresh identification on each build, but only rewrite when content changes.
execute_process(COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY "${SOURCE_DIR}" OUTPUT_VARIABLE revision
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    set(revision "unknown")
endif()
string(REGEX REPLACE "[^A-Za-z0-9._/+~-]" "_" revision "${revision}")
string(SUBSTRING "${revision}" 0 40 revision)
set(content "#ifndef APP_BUILD_INFO_H\n#define APP_BUILD_INFO_H\n#define APP_FIRMWARE_VERSION \"${revision}\"\n#endif\n")
set(old "")
if(EXISTS "${OUTPUT_FILE}")
    file(READ "${OUTPUT_FILE}" old)
endif()
if(NOT old STREQUAL content)
    file(WRITE "${OUTPUT_FILE}" "${content}")
endif()
