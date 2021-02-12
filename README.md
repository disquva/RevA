# RevA

**Host MCU: STM32L071CZYx**

## Code location
[Core/Src](Core/Src)

## Proposed LR1110 firmware update process

*Step 1* and *Step 2* are individually built and sequentially executed on MCU

* Step 1
    * Reboot LR1110 into bootloader 
    * Erase the chip
    * Upload first chunk of firmware (256 Bytes-multiplies long)
    * Note the final offset
* Step 2
    * Reboot LR1110 into bootloader
    * Upload second chunk of firmware starting with noted offset



