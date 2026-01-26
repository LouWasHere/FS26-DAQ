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
#include "lr1121_ping_pong.h"

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
    printf("[LORA] Initializing LR1121 for TX...\n");
    
    lora_init_io_context(&lr1121);
    lora_init_io(&lr1121);
    lora_spi_init(&lr1121);

    printf("[LORA] LR11XX driver version: %s\n", lr11xx_driver_version_get_version_string());

    lora_system_init(&lr1121);
    lora_print_version(&lr1121);
    lora_radio_init(&lr1121);
    
    lora_init_irq(&lr1121, &isr);

    // Only enable TX_DONE interrupt
    ASSERT_LR11XX_RC(lr11xx_system_set_dio_irq_params(&lr1121, LR11XX_SYSTEM_IRQ_TX_DONE, 0));
    ASSERT_LR11XX_RC(lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_ALL_MASK));

    printf("[LORA] TX initialization complete\n");
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
        printf("[LORA] Error: data length %d exceeds max %d\n", length, PAYLOAD_LENGTH);
        return false;
    }

    tx_done_flag = false;
    tx_count++;

    // Write data to radio buffer
    ASSERT_LR11XX_RC(lr11xx_regmem_write_buffer8(&lr1121, data, length));
    
    // Start transmission
    ASSERT_LR11XX_RC(lr11xx_radio_set_tx(&lr1121, 0));

    // Wait for TX to complete (polling with timeout)
    uint32_t timeout_ms = 5000;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    
    while (!tx_done_flag) {
        // Check IRQ register directly as backup
        lr11xx_system_irq_mask_t irq_status;
        lr11xx_system_get_irq_status(&lr1121, &irq_status);
        
        if (irq_status & LR11XX_SYSTEM_IRQ_TX_DONE) {
            tx_done_flag = true;
            break;
        }
        
        if ((to_ms_since_boot(get_absolute_time()) - start) > timeout_ms) {
            printf("[LORA] TX timeout!\n");
            return false;
        }
        
        sleep_ms(1);
    }

    // Clear the IRQ
    ASSERT_LR11XX_RC(lr11xx_system_clear_irq_status(&lr1121, LR11XX_SYSTEM_IRQ_TX_DONE));
    
    printf("[LORA] TX #%lu complete (%d bytes)\n", tx_count, length);
    return true;
}

/**
 * @brief Get the current TX packet count
 */
uint32_t lora_get_tx_count(void)
{
    return tx_count;
}