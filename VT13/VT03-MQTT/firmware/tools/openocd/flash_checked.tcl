# Guarded STM32F407 programming, called only by an explicit just flash operation.
# Definitions alone do not connect. Errors stop before later erase/reset/run steps.

proc fw_check_target {} {
    # RM0090 DBGMCU_IDCODE: low 12 bits identify the F405/407 family (shared ID).
    # STM32F405/407 datasheet: 16-bit flash size at 0x1FFF7A22 is in KiB.
    set device_id [expr {[mrw 0xE0042000] & 0xfff}]
    set flash_kib [mrh 0x1FFF7A22]
    echo [format "FIRMWARE_TARGET id=0x%03x flash_kib=%d" $device_id $flash_kib]
    if {$device_id != 0x413 || $flash_kib != 1024} {
        error "Target mismatch: expected F405/407 family ID 0x413 and 1024 KiB. No erase requested."
    }
}

proc fw_program {} {
    global FW_ELF FW_RUN_AFTER
    if {[catch {
        if {![info exists FW_ELF] || ![file isfile $FW_ELF]} {
            error "Missing firmware ELF; no target connection requested."
        }
        if {![info exists FW_RUN_AFTER] || ($FW_RUN_AFTER ne "0" && $FW_RUN_AFTER ne "1")} {
            error "FW_RUN_AFTER must explicitly be 0 or 1."
        }
        init
        fw_check_target
        # Clear stale core state only after matching the target. Avoid reset init:
        # the vendor reset-init hook changes clocks for faster programming.
        reset halt
        fw_check_target
        flash write_image erase $FW_ELF
        verify_image $FW_ELF
        echo "FIRMWARE_VERIFY_OK"
        if {$FW_RUN_AFTER == 1} {
            reset run
        }
        echo "FIRMWARE_FLASH_OK"
    } failure]} {
        echo "FIRMWARE_ERROR: $failure"
        # shutdown error exits nonzero without retry, reset, unlock or resume.
        shutdown error
        return
    }
    shutdown
}
