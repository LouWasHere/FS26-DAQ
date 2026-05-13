/*!
 * @file      lr1121_tx.c
 *
 * @brief     LoRa TX-only broadcast for LR11xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 * Modified for TX-only operation.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lr1121_tx.h"
#include "safe_print.h"
#include "gpio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static volatile bool tx_done_flag = false;
static uint32_t tx_count = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS -------------------------------------------------------
 */

static void isr(uint gpio, uint32_t events) {
    tx_done_flag = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS --------------------------------------------------------
 */

/**
 * @brief Initialize the LR1121 radio for TX-only operation
 */
void lora_tx_init(void)
{
    safe_printf("[LORA] Initializing LR1121 for TX...\n");
    
    lora_init_io_context(&lr1121);
    lora_init_io(&lr1121);
    lora_spi_init(&lr1121);

    safe_printf("[LORA] LR11XX driver version: %s\n", lr11xx_driver_version_get_version_string());

    lora_system_init(&lr1121);
    lora_print_version(&lr1121);
    lora_radio_init(&lr1121);
    
    lora_init_irq(&lr1121, &isr);

    // Only enable TX_DONE interrupt
    ASSERT_LR11XX_RC(lr11xx_system_set_dio_irq_params(&lr1121, LR11XX_SYSTEM_IRQ_TX_DONE, 0));
    ASSERT_LR11XX_RC(lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK));

    safe_printf("[LORA] TX initialization complete\n");
}

/**
 * @brief Send data over LoRa (blocking until TX complete)
 * 
 * @param data Pointer to data buffer to send
 * @param length Length of data in bytes (max PAYLOAD_LENGTH)
 * @return true if TX completed successfully, false on error
 */
bool lora_send(const uint8_t* data, uint8_t length)
{
    if (length > PAYLOAD_LENGTH) {
        printf("[DBG] TX: payload too large (%u > %u)\n", length, PAYLOAD_LENGTH);
        return false;
    }

    tx_done_flag = false;
    tx_count++;
    printf("[DBG] TX #%lu: Starting send, data_len=%u\n", tx_count, length);
    
    // Clear any pending errors and IRQs
    lr11xx_system_clear_errors(&lr1121);
    lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK);
    
    // Re-enable TCXO with longer timeout (500 * 30.52µs = ~15ms)
    // This is needed because TCXO may have stopped in standby
    lr11xx_system_set_tcxo_mode(&lr1121, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 500);
    
    // Wait for TCXO to stabilize
    sleep_ms(5);
    
    // Clear errors that may have been set during TCXO startup
    lr11xx_system_clear_errors(&lr1121);
    
    // Set packet type (required before TX after fallback to standby)
    lr11xx_radio_set_pkt_type(&lr1121, PACKET_TYPE);
    
    // Set RF frequency
    lr11xx_radio_set_rf_freq(&lr1121, RF_FREQ_IN_HZ);
    
    // Re-apply LoRa modulation params
    lr11xx_radio_mod_params_lora_t mod_params = {
        .sf   = LORA_SPREADING_FACTOR,
        .bw   = LORA_BANDWIDTH,
        .cr   = LORA_CODING_RATE,
        .ldro = 0
    };
    lr11xx_radio_set_lora_mod_params(&lr1121, &mod_params);
    
    // Re-apply LoRa packet params
    lr11xx_radio_pkt_params_lora_t pkt_params = {
        .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
        .header_type          = LORA_PKT_LEN_MODE,
        .pld_len_in_bytes     = PAYLOAD_LENGTH,
        .crc                  = LORA_CRC,
        .iq                   = LORA_IQ,
    };
    lr11xx_radio_set_lora_pkt_params(&lr1121, &pkt_params);

    // Write data to radio buffer (pad to PAYLOAD_LENGTH)
    uint8_t tx_buffer[PAYLOAD_LENGTH] = {0};
    memcpy(tx_buffer, data, length);
    
    lr11xx_status_t rc = lr11xx_regmem_write_buffer8(&lr1121, tx_buffer, PAYLOAD_LENGTH);
    if (rc != LR11XX_STATUS_OK) {
        printf("[DBG] write_buffer failed: %d\n", rc);
        return false;
    }
    
    // Check for errors before TX
    uint16_t sys_errors;
    lr11xx_system_get_errors(&lr1121, &sys_errors);
    if (sys_errors != 0) {
        printf("[DBG] Pre-TX SysErr: 0x%04X\n", sys_errors);
        lr11xx_system_clear_errors(&lr1121);
    }
    
    // Get radio status before TX
    lr11xx_system_irq_mask_t irq_status_before;
    lr11xx_system_get_status(&lr1121, NULL, NULL, &irq_status_before);
    printf("[DBG] Radio status before TX: irq=0x%08lX\n", (unsigned long)irq_status_before);
    
    // Start transmission
    rc = lr11xx_radio_set_tx(&lr1121, 0);
    if (rc != LR11XX_STATUS_OK) {
        printf("[DBG] set_tx failed: %d\n", rc);
        return false;
    }
    printf("[DBG] TX: Radio set to TX mode\n");
    
    // Check radio status immediately after set_tx
    sleep_ms(10);
    lr11xx_system_irq_mask_t irq_status_after;
    lr11xx_system_get_status(&lr1121, NULL, NULL, &irq_status_after);
    printf("[DBG] Radio status after set_tx: irq=0x%08lX\n", (unsigned long)irq_status_after);

    // Wait for TX to complete (polling with timeout)
    uint32_t timeout_ms = 2000;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t poll_count = 0;
    
    while (!tx_done_flag) {
        lr11xx_system_irq_mask_t irq_status;
        lr11xx_system_get_irq_status(&lr1121, &irq_status);
        poll_count++;
        
        if (irq_status & LR11XX_SYSTEM_IRQ_TX_DONE) {
            printf("[DBG] TX: TX_DONE IRQ detected after %lu polls\n", poll_count);
            tx_done_flag = true;
            break;
        }
        
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start;
        if (elapsed > timeout_ms) {
            printf("[DBG] TX timeout after %lums (%lu polls): irq_status=0x%08lX\n", 
                   elapsed, poll_count, (unsigned long)irq_status);
            lr11xx_system_clear_errors(&lr1121);
            lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK);
            return false;
        }
        
        sleep_ms(1);
    }

    // Clear ALL IRQs after TX complete
    lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK);
    printf("[DBG] TX #%lu: TX complete!\n", tx_count);
    
    return true;
}

/**
 * @brief Get the current TX packet count
 */
uint32_t lora_get_tx_count(void)
{
    return tx_count;
}