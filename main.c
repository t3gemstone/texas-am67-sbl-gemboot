/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "drivers/ddr/v1/ddr.h"
#include "drivers/gpio/v0/gpio.h"
#include "drivers/hw_include/j722s/cslr_soc_defines.h"
#include "kernel/dpl/DebugP.h"
#include "kernel/dpl/SystemP.h"
#include "ti_dpl_config.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/device_manager/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/pinmux.h>
#include <drivers/gtc.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_buf_io.h>
#include <drivers/gpio.h>
#include <stdbool.h>

#define BOOTLOADER_UART_STATUS_LOAD_SUCCESS           (0x53554343) /* SUCC */
#define BOOTLOADER_UART_STATUS_LOAD_CPU_FAIL          (0x4641494C) /* FAIL */
#define BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED (0x45584344) /* EXCD */

#define BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS          (5)
#define BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH  (4) /* bytes */
#define BOOTLOADER_APP_IMAGE_LOADED                   (1)

#define BOOTLOADER_REPORT_STATUS                      (0)

#define BYTE_TO_MB(x)                                 ((x)/1024/1024)

#define K3_BOOT_PARAM_TABLE_INDEX_OCRAM		          (0x7000F290U) /* Uboot decides bootmedia from this address */

typedef enum
{
    K3_PRIMARY_BOOTMODE,
    K3_BACKUP_BOOTMODE,
    K3_RAM_BOOTMODE,
    K3_USE_CUSTOM_BOOTMODE,
    K3_INVALID_BOOTMODE,
} Bootmode;

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x100000) /* Max size of the file that xmodem can receive */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
uint8_t socCpuCores[CSL_CORE_ID_MAX]    = {0};

extern Bootloader_MemArgs gBootloader0Args;
extern Bootloader_MemArgs gBootloader1Args;
extern Bootloader_MemArgs gBootloader2Args;

Bootloader_Handle bootHandle;
Bootloader_CpuInfo bootCpuInfo[CSL_CORE_ID_MAX];

typedef enum
{
    BOOTLOADER_PHASE_LINUX_APPIMAGE,
    BOOTLOADER_PHASE_UBOOT_IMAGE,
    BOOTLOADER_PHASE_COMPLETED,
} Bootloader_Phase;

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void Set_Uboot_Spl_Bootmode(Bootmode mode)
{
    uint32_t* reg = (uint32_t*)K3_BOOT_PARAM_TABLE_INDEX_OCRAM;
    if(mode < K3_INVALID_BOOTMODE && mode >= 0)
    {
        *reg = mode;
    }
    else
    {
        *reg = K3_RAM_BOOTMODE;
    }
}

int32_t App_loadImages(void* gAppImageBuf)
{
	int32_t status = SystemP_FAILURE;

    Bootloader_BootImageInfo bootImageInfo;
	Bootloader_Params bootParams;
    Bootloader_Config *bootConfig;

    Bootloader_Params_init(&bootParams);
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootParams.bufIoTempBuf     = gAppImageBuf;
    bootParams.bufIoTempBufSize = BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
    bootParams.bufIoDeviceIndex = COMM_UART;
    bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

    bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);

    if(bootHandle != NULL)
    {
        bootConfig = (Bootloader_Config *)bootHandle;
        bootConfig->coresPresentMap = 0;
        status = Bootloader_parseMultiCoreAppImage(&bootHandle, &bootImageInfo);

        if (status != SystemP_SUCCESS)
        {
            DebugP_log("Bootloader_parseMultiCoreAppImage failed!\r\n");
        }

        /* Load CPUs */
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            DebugP_log("MCU reset isolation is not active!\r\n");
            if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_MCU_R5FSS0_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_MCU_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_MCU_R5FSS0_0);
                status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_MCU_R5FSS0_0]));
                Bootloader_profileAddCore(CSL_CORE_ID_MCU_R5FSS0_0);
                socCpuCores[CSL_CORE_ID_MCU_R5FSS0_0] = BOOTLOADER_APP_IMAGE_LOADED;
                bootCpuInfo[CSL_CORE_ID_MCU_R5FSS0_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_MCU_R5FSS0_0];
            }
        }
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_WKUP_R5FSS0_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_WKUP_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_WKUP_R5FSS0_0);
            status = Bootloader_loadSelfCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_WKUP_R5FSS0_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_WKUP_R5FSS0_0);
        }
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_MAIN_R5FSS0_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_MAIN_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_MAIN_R5FSS0_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_MAIN_R5FSS0_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_MAIN_R5FSS0_0);
            socCpuCores[CSL_CORE_ID_MAIN_R5FSS0_0] = BOOTLOADER_APP_IMAGE_LOADED;
            bootCpuInfo[CSL_CORE_ID_MAIN_R5FSS0_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_MAIN_R5FSS0_0];
        }
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_C75SS0_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_C75SS0_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS0_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_C75SS0_0);
            socCpuCores[CSL_CORE_ID_C75SS0_0] = BOOTLOADER_APP_IMAGE_LOADED;
            bootCpuInfo[CSL_CORE_ID_C75SS0_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS0_0];
        }
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_C75SS1_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_C75SS1_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS1_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_C75SS1_0);
            socCpuCores[CSL_CORE_ID_C75SS1_0] = BOOTLOADER_APP_IMAGE_LOADED;
            bootCpuInfo[CSL_CORE_ID_C75SS1_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_C75SS1_0];
        }
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0)))
		{
			bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS0_0);
			status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_A53SS0_0);
            socCpuCores[CSL_CORE_ID_A53SS0_0] = BOOTLOADER_APP_IMAGE_LOADED;
            bootCpuInfo[CSL_CORE_ID_A53SS0_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0];
		}
        if((SystemP_SUCCESS == status) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS1_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS1_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS1_0]));
            Bootloader_profileAddCore(CSL_CORE_ID_A53SS1_0);
            socCpuCores[CSL_CORE_ID_A53SS1_0] = BOOTLOADER_APP_IMAGE_LOADED;
            bootCpuInfo[CSL_CORE_ID_A53SS1_0] = bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS1_0];
        }
    }

    return status;
}

bool Verify_Uboot_Img(uint8_t* buf)
{
    uint8_t uboot_img_magic[] = { 0xD0, 0x0D, 0xFE, 0xED };

    if (memcmp(buf, &uboot_img_magic, sizeof uboot_img_magic) == 0)
    {
        DebugP_log("U-boot.img verified");
        return true;
    }

    DebugP_log("Invalid U-boot.img");
    return false;
}

void Report_Process_Result(int32_t result)
{
#if (BOOTLOADER_REPORT_STATUS== 1)
    static uint8_t const success[] = { 0xDE, 0xDE, 0xDE, 0xD1 };
    static uint8_t const failure[] = { 0xBA, 0xBA, 0xDE, 0xD1 };

    switch (state) 
    {
        case SystemP_SUCCESS:
            Bootloader_xmodemTransmit(COMM_UART, (uint8_t*)success, sizeof success);
            break;
        case SystemP_FAILURE:
            Bootloader_xmodemTransmit(COMM_UART, (uint8_t*)failure, sizeof failure);
            break;
    }
#endif
}

int main()
{
    int32_t status;
    uint32_t gpioBaseAddr, pinNum;

    Bootloader_Phase phase = BOOTLOADER_PHASE_LINUX_APPIMAGE;

    Bootloader_socWaitForFWBoot();
    status = Bootloader_socOpenFirewalls();

    DebugP_assertNoLog(status == SystemP_SUCCESS);

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    // Close red led
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(RED_LED_BASE_ADDR);
    pinNum = RED_LED_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, RED_LED_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    // open green led
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GREEN_LED_BASE_ADDR);
    pinNum       = GREEN_LED_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, GREEN_LED_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    DebugP_log("\r\nTexas AM67 SBL Gemboot Started...!\r\n");

    if(SystemP_SUCCESS == status)
    {
        uint32_t fileSize = 0;
        uint8_t* bufStart = NULL;

        while(true)
        {
            switch (phase) {
                case BOOTLOADER_PHASE_LINUX_APPIMAGE:
                {
                    DebugP_log("Waiting for the transfer of the file linux.appimage.hs_fs... (Method: XMODEM, BAUD: 3000000)\r\n");
                    bufStart = gAppImageBuf + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
                    status = Bootloader_xmodemReceive(COMM_UART, bufStart, BOOTLOADER_APPIMAGE_MAX_FILE_SIZE, &fileSize);
                    if(SystemP_SUCCESS == status && fileSize == BOOTLOADER_APPIMAGE_MAX_FILE_SIZE)
                    {
                        status = SystemP_FAILURE;
                        DebugP_log("File size is larger than the RX buffer size (%d MB), operation failed!\r\n", BYTE_TO_MB(BOOTLOADER_APPIMAGE_MAX_FILE_SIZE));
                    }

                    if(SystemP_SUCCESS == status)
                    {
                        status = App_loadImages(bufStart);

                        if(status != SystemP_SUCCESS)
                        {
                            DebugP_log("The download of linux.appimage.hs_fs failed\r\n");
                            Report_Process_Result(SystemP_FAILURE);
                        }
                        else 
                        {
                            DebugP_log("The download of linux.appimage.hs_fs was successful.\r\n");
                            phase = BOOTLOADER_PHASE_UBOOT_IMAGE;
                            Report_Process_Result(SystemP_SUCCESS);
                        }
                    }
                    else 
                    {
                        DebugP_log("XMODEM reception of linux.appimage.hs_fs failed!\r\n");
                        Report_Process_Result(SystemP_FAILURE);
                    }
                } break;
                case BOOTLOADER_PHASE_UBOOT_IMAGE:
                {
                    DebugP_log("Waiting for the transfer of the file u-boot.img... (Method: XMODEM, BAUD: 3000000)\r\n");
                    bufStart = gAppImageBuf;
                    status = Bootloader_xmodemReceive(COMM_UART, bufStart, BOOTLOADER_APPIMAGE_MAX_FILE_SIZE, &fileSize);
                    if(SystemP_SUCCESS == status && fileSize == BOOTLOADER_APPIMAGE_MAX_FILE_SIZE)
                    {
                        status = SystemP_FAILURE;
                        DebugP_log("File size is larger than the RX buffer size (%d MB), operation failed!\r\n", BYTE_TO_MB(BOOTLOADER_APPIMAGE_MAX_FILE_SIZE));
                        Report_Process_Result(SystemP_FAILURE);
                    }

                    if(SystemP_SUCCESS == status && false == Verify_Uboot_Img(bufStart))
                    {
                        DebugP_log("The received u-boot.img file is invalid.\r\n");
                        status = SystemP_FAILURE;
                        Report_Process_Result(SystemP_FAILURE);
                    }

                    if(SystemP_SUCCESS == status)
                    {
                        DebugP_log("u-boot.img was successfully received.\r\n");
                        Report_Process_Result(SystemP_SUCCESS);
                        phase = BOOTLOADER_PHASE_COMPLETED;
                    }
                    else 
                    {
                        DebugP_log("XMODEM reception of u-boot.img failed.\r\n");
                        Report_Process_Result(SystemP_FAILURE);
                    }
                } break;
                
                case BOOTLOADER_PHASE_COMPLETED:
                {
                    DebugP_log("Core A53SS0_0 is starting...\r\n");
                    // ClockP_sleep(BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS);

                    Set_Uboot_Spl_Bootmode(K3_RAM_BOOTMODE);
                    status = Bootloader_runCpu(bootHandle, &bootCpuInfo[CSL_CORE_ID_A53SS0_0]);
    
                    if (SystemP_SUCCESS != status)
                    {
                        DebugP_log("Core A53SS0_0 could not be started!\r\n");
                        Report_Process_Result(SystemP_FAILURE);
                    }
                    else 
                    {
                        DebugP_log("Core A53SS0_0 has been started!\r\n");
                        Report_Process_Result(SystemP_SUCCESS);
                        Bootloader_JumpSelfCpu();
                    }
                } break;
            }
        }
    }

    Board_driversClose();
    Drivers_close();

    /* Call DPL deinit to close the tick timer and disable interrupts before jumping to DM*/
    Dpl_deinit();
    System_deinit();

    Bootloader_JumpSelfCpu();
    return 0;
}
