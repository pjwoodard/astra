/* NUCLEO-H533RE: STM32H533RET6, Cortex-M33 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  /* SRAM1 + SRAM2 + SRAM3 are contiguous from 0x20000000 (272K total on the
     H533). Backup SRAM (BKPSRAM, separate window) is not used here.
     Assumes a non-TrustZone (TZEN=0) image — the whole map is non-secure. */
  RAM   : ORIGIN = 0x20000000, LENGTH = 272K
}
