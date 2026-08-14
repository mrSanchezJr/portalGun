#include "eeprom_emu.h"
#include "config.h"
#include <Arduino.h>
#include <stm32f1xx_hal.h>

static const uint32_t PAGE_STATUS_ACTIVE   = 0xEEEEEEEEUL;
static const uint32_t PAGE_STATUS_CLEAN    = 0xFFFFFFFFUL;
static const uint32_t PAGE_STATUS_TRANSFER = 0xBEEFBEEFUL;

static const uint32_t HEADER_SIZE = 4;
static const uint32_t MAX_CELLS = (EEPROM_PAGE_SIZE - HEADER_SIZE) / 4;

static HAL_StatusTypeDef flash_erase_page(uint32_t page_addr) {
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = page_addr;
    erase.NbPages     = 1;
    uint32_t page_err = 0;
    return HAL_FLASHEx_Erase(&erase, &page_err);
}

static HAL_StatusTypeDef flash_write_word(uint32_t addr, uint32_t data) {
    HAL_StatusTypeDef s;
    s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint16_t)(data & 0xFFFF));
    if (s != HAL_OK) return s;
    s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2, (uint16_t)(data >> 16));
    return s;
}

static inline uint32_t flash_read_word(uint32_t addr) {
    return *((volatile uint32_t*)addr);
}

static int8_t find_active_page(void) {
    uint32_t s0 = flash_read_word(EEPROM_PAGE0_BASE);
    uint32_t s1 = flash_read_word(EEPROM_PAGE1_BASE);
    if (s0 == PAGE_STATUS_ACTIVE) return 0;
    if (s1 == PAGE_STATUS_ACTIVE) return 1;
    return -1;
}

static inline uint32_t page_base(int8_t idx) {
    return (idx == 0) ? EEPROM_PAGE0_BASE : EEPROM_PAGE1_BASE;
}

static bool page_find(int8_t page_idx, uint16_t addr, uint16_t &out) {
    uint32_t base = page_base(page_idx) + HEADER_SIZE;
    bool found = false;
    for (uint32_t i = 0; i < MAX_CELLS; i++) {
        uint32_t cell = flash_read_word(base + i * 4);
        uint16_t cell_addr = (uint16_t)(cell >> 16);
        uint16_t cell_data = (uint16_t)(cell & 0xFFFF);
        if (cell_addr == addr) {
            out = cell_data;
            found = true;
        }
        if (cell == 0xFFFFFFFF) break;
    }
    return found;
}

static uint32_t page_find_free(int8_t page_idx) {
    uint32_t base = page_base(page_idx) + HEADER_SIZE;
    for (uint32_t i = 0; i < MAX_CELLS; i++) {
        uint32_t cell = flash_read_word(base + i * 4);
        if (cell == 0xFFFFFFFF) return base + i * 4;
    }
    return 0;
}

static void page_transfer(int8_t old_page) {
    int8_t new_page = (old_page == 0) ? 1 : 0;
    uint32_t new_base = page_base(new_page);

    HAL_FLASH_Unlock();

    flash_erase_page(new_base);
    flash_write_word(new_base, PAGE_STATUS_ACTIVE);

    static const uint16_t known_addrs[] = { EE_ADDR_COUNTER, EE_ADDR_BRIGHTNESS };
    uint32_t dst = new_base + HEADER_SIZE;

    for (uint8_t k = 0; k < sizeof(known_addrs)/sizeof(known_addrs[0]); k++) {
        uint16_t val = 0;
        if (page_find(old_page, known_addrs[k], val)) {
            uint32_t cell = ((uint32_t)known_addrs[k] << 16) | val;
            flash_write_word(dst, cell);
            dst += 4;
        }
    }

    flash_erase_page(page_base(old_page));

    HAL_FLASH_Lock();
}

void eeprom_init(void) {
    int8_t active = find_active_page();

    if (active == -1) {
        HAL_FLASH_Unlock();
        flash_erase_page(EEPROM_PAGE0_BASE);
        flash_erase_page(EEPROM_PAGE1_BASE);
        flash_write_word(EEPROM_PAGE0_BASE, PAGE_STATUS_ACTIVE);
        HAL_FLASH_Lock();
    }

    uint32_t s0 = flash_read_word(EEPROM_PAGE0_BASE);
    uint32_t s1 = flash_read_word(EEPROM_PAGE1_BASE);
    if (s0 == PAGE_STATUS_TRANSFER) {
        HAL_FLASH_Unlock();
        flash_erase_page(EEPROM_PAGE0_BASE);
        HAL_FLASH_Lock();
        if (s1 != PAGE_STATUS_ACTIVE) {
            HAL_FLASH_Unlock();
            flash_erase_page(EEPROM_PAGE1_BASE);
            flash_write_word(EEPROM_PAGE1_BASE, PAGE_STATUS_ACTIVE);
            HAL_FLASH_Lock();
        }
    } else if (s1 == PAGE_STATUS_TRANSFER) {
        HAL_FLASH_Unlock();
        flash_erase_page(EEPROM_PAGE1_BASE);
        HAL_FLASH_Lock();
    }
}

bool eeprom_read(uint16_t addr, uint16_t &data) {
    int8_t active = find_active_page();
    if (active == -1) return false;
    return page_find(active, addr, data);
}

void eeprom_write(uint16_t addr, uint16_t data) {
    int8_t active = find_active_page();
    if (active == -1) return;

    uint32_t free_cell = page_find_free(active);

    if (free_cell == 0) {
        page_transfer(active);
        active = find_active_page();
        if (active == -1) return;
        free_cell = page_find_free(active);
        if (free_cell == 0) return;
    }

    uint32_t cell = ((uint32_t)addr << 16) | (uint32_t)data;
    HAL_FLASH_Unlock();
    flash_write_word(free_cell, cell);
    HAL_FLASH_Lock();
}
