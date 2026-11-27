*** Settings ***
Documentation     Boot test of the CubeMX-generated safety-node image (STM32G474RE, bare-metal super-loop) in Renode.
...
...               The probes are installed by safety-node.resc and log "PROBE <name>" lines. Nothing in the
...               firmware is modified or instrumented. RCC, PWR and FLASH_ACR are stubs (Renode has no
...               STM32G4 platform); see tests/renode/README.md for what is real and what is stubbed.
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Reset Emulation
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${RESC}           ${CURDIR}/safety-node.resc
${ELF}            ${CURDIR}/../../cubemx/build/Debug/safety-node.elf
${RCC_CR}         0x40021000
${RCC_CFGR}       0x40021008
${RCC_PLLCFGR}    0x4002100C
${RUN_SECONDS}    2

*** Keywords ***
Load Image
    [Documentation]    Load the ELF with the probes and attach the log tester (emulation stays paused;
    ...                time only advances in the Run keywords below).
    Execute Command    $elf=@${ELF}
    Execute Command    include @${RESC}
    Create Log Tester  0

Run For
    [Arguments]        ${seconds}
    Execute Command    emulation RunFor "${seconds}"

Probe Should Appear
    [Arguments]        ${probe}
    Wait For Log Entry    PROBE ${probe}    timeout=0

Read Register
    [Arguments]        ${address}
    ${value}=          Execute Command    sysbus ReadDoubleWord ${address}
    ${number}=         Evaluate    int($value.strip(), 16)
    RETURN             ${number}

Watchdog Refresh Should Happen Within
    [Documentation]    One-shot probe on HAL_IWDG_Refresh (a permanent one would log millions of lines):
    ...                armed now, it must fire within the given emulated time. The tag keeps every
    ...                probe line unique.
    [Arguments]        ${tag}    ${seconds}
    Execute Command    sysbus.cpu AddHook `sysbus GetSymbolAddress "HAL_IWDG_Refresh"` "self.WarningLog('PROBE IWDG_REFRESH ${tag}'); self.RemoveHooksAt(self.PC)"
    Run For            ${seconds}
    Probe Should Appear    IWDG_REFRESH ${tag}

*** Test Cases ***
Core Reaches Main And Completes Clock Init
    [Documentation]    HSE 24 MHz crystal + PLL (M=6, N=85, R=2: 170 MHz). The status bits come from the
    ...                RCC stub, so this checks that the firmware asked for the right configuration and
    ...                that every HAL status check passed (no Error_Handler), not the clock tree itself.
    Load Image
    Run For            0.05
    Probe Should Appear    Reset_Handler
    Probe Should Appear    main
    # MX_GPIO_Init is the first call after SystemClock_Config(); reaching it means every
    # HAL_RCC_OscConfig / HAL_RCC_ClockConfig status check passed.
    Probe Should Appear    clock_init_done
    ${cr}=             Read Register    ${RCC_CR}
    ${cfgr}=           Read Register    ${RCC_CFGR}
    ${pll}=            Read Register    ${RCC_PLLCFGR}
    Should Be Equal As Integers    ${{($cr >> 16) & 1}}     1    HSEON not set
    Should Be Equal As Integers    ${{($cr >> 18) & 1}}     0    HSEBYP set: the NUCLEO-G474RE uses the 24 MHz crystal
    Should Be Equal As Integers    ${{$cfgr & 0x3}}         3    SW is not PLL
    Should Be Equal As Integers    ${{($pll >> 0) & 0x3}}   3    PLLSRC is not HSE
    Should Be Equal As Integers    ${{($pll >> 4) & 0xF}}   5    PLLM is not /6
    Should Be Equal As Integers    ${{($pll >> 8) & 0x7F}}  85   PLLN is not 85
    Should Be Equal As Integers    ${{($pll >> 25) & 0x3}}  0    PLLR is not /2

FDCAN Nominal Bit Timing Is Programmed
    [Documentation]    Prescaler 20, TSEG1 14, TSEG2 2, SJW 2 in HAL terms = NBRP 19, NTSEG1 13, NTSEG2 1, NSJW 1.
    Load Image
    Run For            0.05
    Probe Should Appear    NBTP fdcan1 nbrp=19 ntseg1=13 ntseg2=1 nsjw=1

Super Loop Runs And Refreshes The Watchdog
    Load Image
    Run For            0.05
    # Five consecutive time slices, each of which must contain a refresh from the super-loop.
    FOR    ${i}    IN RANGE    5
        Watchdog Refresh Should Happen Within    ${i}    0.05
    END

No Reset And No Fault Handler Within Two Seconds
    [Documentation]    Any entry into a fault/error handler or a machine reset (IWDG expiry, system reset)
    ...                fails the test; the super-loop must still be refreshing the watchdog at the end.
    Load Image
    Run For            ${RUN_SECONDS}
    Should Not Be In Log    PROBE FAULT
    Watchdog Refresh Should Happen Within    end    0.05
