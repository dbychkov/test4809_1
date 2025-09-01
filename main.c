/*
 * SD Card FAT32 Root Directory Reader for ATmega4809 (DIP-40)
 * Outputs root directory entries to serial console (USART0)
 * No Arduino libraries or dependencies.
 *
 * Uses SPI to communicate with SD Card.
 * Assumes SD Card is formatted as FAT32.
 * Pin configuration may need adjustment for your hardware.
 * This example assumes only the first cluster of the root directory is displayed (most cards will show many entries).
 * No error handling for long file names (LFN) or subdirectories.
 * Use a USB-to-Serial adapter connected to TXD (PC0) to view output.
 * 
 * Pin mapping (change as needed):
 *   MOSI: PA0
 *   MISO: PA1
 *   SCK:  PA2
 *   SS:   PA3
 *   TXD (USART0): PC0
 *   RXD (USART0): PC1
 *
 * Compile with avr-gcc, flash with avrdude.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define SD_CS_LOW()  (PORTA.OUTCLR = PIN3_bm)
#define SD_CS_HIGH() (PORTA.OUTSET = PIN3_bm)

// Sector size
#define SECTOR_SIZE 512

// USART0 baudrate settings
#define BAUD_RATE 9600
#define BAUD_SETTING ((F_CPU / (16UL * BAUD_RATE)) - 1)

// ----------- USART0 Functions -----------
void usart0_init(void) {
    // Set TXD (PC0) as output
    PORTC.DIRSET = PIN0_bm;
    // Set RXD (PC1) as input
    PORTC.DIRCLR = PIN1_bm;

    USART0.BAUD = BAUD_SETTING;
    USART0.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;
    USART0.CTRLA = 0;
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void usart0_putc(char c) {
    while (!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = c;
}

void usart0_puts(const char *str) {
    while (*str) usart0_putc(*str++);
}

void usart0_print_hex(uint8_t v) {
    char hex[] = "0123456789ABCDEF";
    usart0_putc(hex[(v >> 4) & 0xF]);
    usart0_putc(hex[v & 0xF]);
}

// ----------- SPI Functions -----------
void spi_init(void) {
    // MOSI: PA0 (output)
    // MISO: PA1 (input)
    // SCK:  PA2 (output)
    // SS:   PA3 (output)
    PORTA.DIRSET = PIN0_bm | PIN2_bm | PIN3_bm;
    PORTA.DIRCLR = PIN1_bm;

    // SPI0: Master, MSB first, Mode 0, Prescaler /4
    SPI0.CTRLA = SPI_ENABLE_bm | SPI_MASTER_bm;
    SPI0.CTRLB = SPI_SSD_bm; // Slave Select Disable (manual SS)
    SPI0.INTCTRL = 0;
}

uint8_t spi_transfer(uint8_t data) {
    SPI0.DATA = data;
    while (!(SPI0.INTFLAGS & SPI_IF_bm));
    return SPI0.DATA;
}

void spi_send_dummy(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) spi_transfer(0xFF);
}

// ----------- SD Card SPI Protocol -----------
uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    SD_CS_LOW();
    spi_transfer(0x40 | cmd);
    spi_transfer(arg >> 24);
    spi_transfer(arg >> 16);
    spi_transfer(arg >> 8);
    spi_transfer(arg);
    spi_transfer(crc);

    // Wait for response (max 8 bytes)
    uint8_t res;
    for (uint8_t i = 0; i < 8; i++) {
        res = spi_transfer(0xFF);
        if ((res & 0x80) == 0) break;
    }
    return res;
}

uint8_t sd_init(void) {
    SD_CS_HIGH();
    spi_send_dummy(10); // 80 clock cycles

    // CMD0: Go idle
    if (sd_send_cmd(0, 0, 0x95) != 0x01) return 0;

    // CMD8: Check voltage range
    if (sd_send_cmd(8, 0x1AA, 0x87) != 0x01) return 0;
    spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF);

    // ACMD41: Initialize card
    uint16_t timeout = 0xFFFF;
    do {
        sd_send_cmd(55, 0, 0x65);
        if (sd_send_cmd(41, 0x40000000, 0x77) == 0x00) break;
    } while (--timeout);
    if (!timeout) return 0;

    // CMD58: Read OCR
    if (sd_send_cmd(58, 0, 0xFD) != 0x00) return 0;
    spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 1;
}

uint8_t sd_read_sector(uint32_t sector, uint8_t *buffer) {
    SD_CS_LOW();
    if (sd_send_cmd(17, sector, 0xFF) != 0x00) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 0;
    }
    // Wait for data token
    uint16_t t = 0xFFFF;
    uint8_t token;
    do {
        token = spi_transfer(0xFF);
        if (token == 0xFE) break;
    } while (--t);
    if (!t) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 0;
    }
    // Read 512 bytes
    for (uint16_t i = 0; i < SECTOR_SIZE; i++) buffer[i] = spi_transfer(0xFF);
    // Ignore CRC
    spi_transfer(0xFF); spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 1;
}

// ----------- FAT32 Structures -----------
typedef struct {
    uint16_t bytes_per_sector;
    uint8_t sectors_per_cluster;
    uint16_t reserved_sector_count;
    uint8_t num_fats;
    uint32_t fat_size;
    uint32_t root_cluster;
    uint32_t fat_start;
    uint32_t data_start;
} fat32_info_t;

uint8_t parse_fat32_info(uint8_t *sector, fat32_info_t *info) {
    info->bytes_per_sector = ((uint16_t)sector[11]) | ((uint16_t)sector[12] << 8);
    info->sectors_per_cluster = sector[13];
    info->reserved_sector_count = ((uint16_t)sector[14]) | ((uint32_t)sector[15] << 8);
    info->num_fats = sector[16];
    info->fat_size = ((uint32_t)sector[36]) | ((uint32_t)sector[37] << 8) | ((uint32_t)sector[38] << 16) | ((uint32_t)sector[39] << 24);
    info->root_cluster = ((uint32_t)sector[44]) | ((uint32_t)sector[45] << 8) | ((uint32_t)sector[46] << 16) | ((uint32_t)sector[47] << 24);
    info->fat_start = info->reserved_sector_count;
    info->data_start = info->fat_start + (info->num_fats * info->fat_size);
    return 1;
}

// ----------- Directory Output -----------
void print_filename(uint8_t *entry) {
    char name[13];
    uint8_t j = 0;
    // 8.3 name
    for (uint8_t i = 0; i < 8; i++) {
        if (entry[i] == ' ') break;
        name[j++] = entry[i];
    }
    if (entry[8] != ' ') {
        name[j++] = '.';
        for (uint8_t i = 8; i < 11; i++) {
            if (entry[i] == ' ') break;
            name[j++] = entry[i];
        }
    }
    name[j] = 0;
    usart0_puts(name);
}

void print_dir_entry(uint8_t *entry) {
    // Only display regular files and directories
    if (entry[0] == 0x00 || entry[0] == 0xE5 || (entry[11] & 0x08)) return;
    usart0_puts("Name: ");
    print_filename(entry);
    usart0_puts(" | Attr: ");
    usart0_print_hex(entry[11]);
    usart0_puts(" | Size: ");

    uint32_t fsize = ((uint32_t)entry[28]) | ((uint32_t)entry[29] << 8) | ((uint32_t)entry[30] << 16) | ((uint32_t)entry[31] << 24);
    char sizebuf[11];
    itoa(fsize, sizebuf, 10);
    usart0_puts(sizebuf);

    usart0_puts("\r\n");
}

// ----------- Main Routine -----------
int main(void) {
    uint8_t sector[SECTOR_SIZE];
    fat32_info_t fat32;
    
    usart0_init();
    spi_init();

    usart0_puts("\r\nInitializing SD Card...\r\n");
    if (!sd_init()) {
        usart0_puts("SD Card init failed!\r\n");
        while (1);
    }
    usart0_puts("SD Card OK.\r\n");

    // Read sector 0 (MBR)
    if (!sd_read_sector(0, sector)) {
        usart0_puts("MBR read error\r\n");
        while (1);
    }

    // Partition 0 start sector
    uint32_t part_start = ((uint32_t)sector[454]) | ((uint32_t)sector[455]<<8) | ((uint32_t)sector[456]<<16) | ((uint32_t)sector[457]<<24);

    // Read boot sector (VBR)
    if (!sd_read_sector(part_start, sector)) {
        usart0_puts("VBR read error\r\n");
        while (1);
    }

    parse_fat32_info(sector, &fat32);

    usart0_puts("FAT32 detected. Root Cluster: ");
    char buf[11];
    itoa(fat32.root_cluster, buf, 10);
    usart0_puts(buf);
    usart0_puts("\r\n");

    // Calculate first root directory sector
    uint32_t first_root_sector = fat32.data_start + (fat32.sectors_per_cluster * (fat32.root_cluster - 2));

    // Read root directory (first cluster only)
    if (!sd_read_sector(first_root_sector, sector)) {
        usart0_puts("Root dir read error\r\n");
        while (1);
    }

    usart0_puts("Root Directory:\r\n");
    for (uint16_t i = 0; i < SECTOR_SIZE; i += 32) {
        print_dir_entry(&sector[i]);
    }

    while (1);
}
