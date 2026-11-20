# Test-only hardware substitutes. Run with OpenOCD noinit and no interface file.
# Every command that could access a target is replaced before fw_program is called.
rename init fw_real_init
rename shutdown fw_real_shutdown
proc init {} {
    echo "MOCK_INIT"
    if {$::SCENARIO eq "init-failure"} { error "mock connection failure" }
}
proc mem2array {name width address count} {
    upvar 1 $name words
    if {$::SCENARIO eq "read-failure"} { error "mock memory read failure" }
    if {$address == 0xE0042000 && $width == 32} {
        set words(0) 0x10000413
        if {$::SCENARIO eq "wrong-id"} { set words(0) 0x419 }
        if {$::SCENARIO eq "changed-id" && $::HALTED} { set words(0) 0x419 }
    } elseif {$address == 0x1FFF7A22 && $width == 16} {
        set words(0) 1024
        if {$::SCENARIO eq "wrong-size"} { set words(0) 512 }
    } else {
        error "unexpected register read"
    }
}
proc reset {mode} {
    echo "MOCK_RESET_$mode"
    if {$::SCENARIO eq "reset-failure"} { error "mock reset failure" }
    set ::HALTED 1
}
proc mrw {address} {
    mem2array words 32 $address 1
    return $words(0)
}
proc mrh {address} {
    mem2array words 16 $address 1
    return $words(0)
}
proc flash {operation erase elf} {
    if {$operation ne "write_image" || $erase ne "erase" || !$::HALTED} {
        error "unexpected write sequence"
    }
    echo "MOCK_WRITE"
    if {$::SCENARIO eq "write-failure"} { error "mock write failure" }
}
proc verify_image {elf} {
    echo "MOCK_VERIFY"
    if {$::SCENARIO eq "verify-failure"} { error "mock verify failure" }
}
proc shutdown {args} {
    echo "MOCK_SHUTDOWN $args"
    if {[llength $args]} { fw_real_shutdown error } else { fw_real_shutdown }
}
set HALTED 0
