/*!
 * @file      lr1121_tx.h
 *
 * @brief     LoRa TX-only broadcast for LR11xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 * Modified for TX-only operation.
 */

#ifndef LR1121_TX_H
#define LR1121_TX_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1121_config.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the LR1121 radio for TX-only operation
 * 
 * Call this once before using lora_send()
 */
void lora_tx_init(void);

/**
 * @brief Send data over LoRa (blocking until TX complete)
 * 
 * @param data Pointer to data buffer to send
 * @param length Length of data in bytes (max PAYLOAD_LENGTH)
 * @return true if TX completed successfully, false on timeout/error
 */
bool lora_send(const uint8_t* data, uint8_t length);

/**
 * @brief Get the current TX packet count
 * 
 * @return Number of packets transmitted since init
 */
uint32_t lora_get_tx_count(void);

#endif // LR1121_TX_H

/* --- EOF ------------------------------------------------------------------ */
