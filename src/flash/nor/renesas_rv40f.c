/***************************************************************************
 *   Copyright (c) 2022 Miloslav Semler                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* 29-Mar-22: Minor changes to make it compile. (KH) */
/* 29-Mar-22: Added 'const struct flash_driver' at the end so that the
              driver can be added to OpenOCD (KH) */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* FACI registers */
#define FACI_FWEPROR 0x4001E416
#define FACI_FENTRYR 0x407FE084
#define FACI_FSTATR 0x407FE080
#define FACI_FASTAT 0x407FE010
#define FACI_FSADDR 0x407FE030
#define FACI_FEADDR 0x407FE034
#define FACI_CMD_AREA 0x407E0000

/* register bits */
#define FACI_FSTATR_FLWEERR 0x0040
#define FACI_FSTATR_PRGSPD 0x0100
#define FACI_FSTATR_ERSSPD 0x0200
#define FACI_FSTATR_DBFULL 0x0400
#define FACI_FSTATR_SUSRDY 0x0800
#define FACI_FSTATR_PRGERR 0x1000
#define FACI_FSTATR_ERSERR 0x2000
#define FACI_FSTATR_ILGLERR 0x4000
#define FACI_FSTATR_FRDY 0x8000
#define FACI_FSTATR_OTERR 0x100000
#define FACI_FSTATR_SECERR 0x200000
#define FACI_FSTATR_FESETERR 0x400000
#define FACI_FSTATR_ILGCOMERR 0x800000

#define FACI_FASTAT_CFAE 0x80
#define FACI_FASTAT_CMDLK 0x10
#define FACI_FASTAT_DFAE 0x08


struct renesas_bank {
    /* bank size in bytes */
    unsigned size;
    /* base address */
    unsigned base;
    /* 1 for data flash or 0 for program flash */
    unsigned is_data;
    bool probed;
};

FLASH_BANK_COMMAND_HANDLER(rv40f_flash_bank_command)
{
    struct renesas_bank *info;

    /* base address */
    unsigned base = strtoul(CMD_ARGV[1], NULL, 16);
    /* memory size */
    unsigned size = strtoul(CMD_ARGV[2], NULL, 16);

    info = malloc(sizeof(struct renesas_bank));
    bank->driver_priv = info;

    info->base = base;
    info->size = size;
    info->is_data = 0;		/* assume program flash initially */
    info->probed = false;

    return ERROR_OK;
}

/* Data polling algorithm */
static int rv40f_busy_wait(struct target *target, int timeout_ms)
{
    int retval = ERROR_OK;
    int ms =0;
    uint32_t reg32;

    do {
        /* read status register FSTATR */
        retval = target_read_u32(target, FACI_FSTATR, &reg32);

        if (retval != ERROR_OK)
            return retval;

        /* No ERASE/PROGRAM operation in progress */
        if(reg32 & FACI_FSTATR_FRDY){
            break;
        }

        usleep(1000);
		++ms;

		/* Polling time exceeded? */
		if (ms > timeout_ms) {
			LOG_ERROR("Waiting on FSTATR->FRDY timed out!");
			return ERROR_FLASH_OPERATION_FAILED;
		}

    } while(1);

    if (retval == ERROR_OK)
		LOG_DEBUG("rv40f_busy_wait() took about %d ms", ms);

    return retval;
}

static int rv40f_unlock(struct target *target)
{
    uint32_t reg32;
    uint8_t reg8;
    int retval;

    LOG_ERROR("Unlocking FACI after lockup.");

    /* we do not need check FRDY flag, as we check it in all functions */
    retval = target_read_u32(target, FACI_FSTATR, &reg32);

    if (retval != ERROR_OK)
		return retval;

    if(reg32 & FACI_FSTATR_ILGLERR){
        retval = target_read_u8(target, FACI_FASTAT, &reg8);

        if (retval != ERROR_OK)
            return retval;

        /* reset FAE and DFAE flags if needed */
        if(reg8 & (FACI_FASTAT_CFAE|FACI_FASTAT_DFAE)){
            reg8 &= ~(FACI_FASTAT_CFAE|FACI_FASTAT_DFAE);

            retval = target_write_u8(target, FACI_FASTAT, reg8);

            if (retval != ERROR_OK)
                return retval;
        }
    }

    /* issue force stop command - 3B */
    retval = target_write_u8(target, FACI_CMD_AREA, 0x3B);

    if (retval != ERROR_OK)
		return retval;

    retval = rv40f_busy_wait(target, 1000);

    return retval;
}

/* wait for complete FACI command and do 
 * cleanup after lockup if needed */
static int rv40f_complete(struct target *target, int timeout)
{
    int retval;
    uint8_t reg8;
    uint32_t reg32;

    /* check CMDLK bit from FASTAT register */
    retval = target_read_u8(target, FACI_FASTAT, &reg8);

    retval = rv40f_busy_wait(target, timeout);

    if (retval != ERROR_OK)
        return retval;

    /* check CMDLK bit from FASTAT register */
    retval = target_read_u8(target, FACI_FASTAT, &reg8);

    if (retval != ERROR_OK)
        return retval;

    /* check CMDLK bit */
    if (reg8 & 0x10){
        rv40f_unlock(target);
    retval = target_read_u32(target, FACI_FSTATR, &reg32);
    retval = target_read_u32(target, 0X407FE078, &reg32);
    retval = target_read_u8(target, 0X10081B0, &reg8);
    retval = target_read_u32(target, 0X407FE014, &reg32);
    retval = target_read_u32(target, 0X407FE018, &reg32);

        retval = ERROR_FLASH_OPERATION_FAILED;
    }

    return retval;
}

/* switch to read mode after erasing or programming */
static int rv40f_to_readonly(struct target *target)
{
    int retval;
    uint32_t reg32;

    retval = target_read_u32(target, FACI_FSTATR, &reg32);

    if (retval != ERROR_OK)
        return retval;

    /* operation in progress */
    if((reg32 & FACI_FSTATR_FRDY) == 0){
        return ERROR_FLASH_OPERATION_FAILED;
    }

    retval = target_write_u16(target, FACI_FENTRYR, 0xAA00);

    return retval;
}

static int rv40f_pe_mode(struct flash_bank *bank)
{
    struct renesas_bank *info = bank->driver_priv;
    int retval;
    uint16_t r16;

    /* unprotect memories from program/erase */
    retval = target_write_u8(bank->target, FACI_FWEPROR, 1);

    if (retval != ERROR_OK)
        return retval;

    /* set P/E mode according to base address (FENTRYR) */
    if(info->is_data){
        /* data flash */
        retval = target_write_u16(bank->target, FACI_FENTRYR, 0xAA80);
    }else{
        /* code flash */
        retval = target_write_u16(bank->target, FACI_FENTRYR, 0xAA01);
    }

    if (retval != ERROR_OK)
        return retval;

    /* check if the things are OK */
    retval = target_read_u16(bank->target, FACI_FENTRYR, &r16);

    if(info->is_data && (r16 & 0xaa80) == 0x80){
        LOG_INFO("FACI in data flash P/E mode ...");
    }else if((r16 & 0xaa01) == 0x1){
        LOG_INFO("FACI in program flash P/E mode ...");
    }else{
        /* 没有进入 P/E 模式 */
        LOG_ERROR("Flash not in P/E mode");
        retval = ERROR_FLASH_OPERATION_FAILED;
    }

    return retval;
}

static int rv40f_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
    struct renesas_bank *info = bank->driver_priv;
    struct target *target = bank->target;
    int retval;
    uint8_t reg8;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = target_read_u8(target, FACI_FASTAT, &reg8);
    /* enter P/E mode */
    retval = rv40f_pe_mode(bank);

    if (retval != ERROR_OK)
        return retval;


    for (unsigned int sector = first ; sector <= last ; sector++) {
        unsigned offset = bank->sectors[sector].offset;

         /* start adddress */
        retval = target_write_u32(bank->target, FACI_FSADDR, offset + info->base);

        if (retval != ERROR_OK)
            return retval;

        /* fire the erase */
        retval = target_write_u8(target, FACI_CMD_AREA, 0x20);

        if (retval != ERROR_OK)
            return retval;

        retval = target_write_u8(target, FACI_CMD_AREA, 0xd0);

        if (retval != ERROR_OK)
            return retval;

        retval = rv40f_complete(target, 500);

        if (retval != ERROR_OK)
            return retval;
    }

    /* switch back to readonly after operation */
    retval = rv40f_to_readonly(target);

    return retval;
}


 __attribute__ ((unused)) static int __rv40f_flash_write(struct target *target, unsigned int count, uint16_t *wa_start, uint16_t *wa_end, uint32_t dst_addr)
{
    unsigned int i = 0;
    __attribute__ ((unused)) uint32_t value = 0;
    uint8_t reg8   = 0;
 __attribute__ ((unused))    uint16_t reg16   = 0;
    uint32_t reg32   = 0;
    int retval;

    target_write_u32(target, FACI_FSADDR, (uint32_t)dst_addr);
    target_write_u8(target, FACI_CMD_AREA, 0XE8);
    target_write_u8(target, FACI_CMD_AREA, count);
#if 0
    target_write_u16(target, FACI_CMD_AREA, 0X5678);
    LOG_ERROR("dbg 2222.2:%d", count);
    target_write_u16(target, FACI_CMD_AREA, 0X8765);
    LOG_ERROR("dbg 2222.3:%d", count);
#endif

    for (; i < count; i++)
    {
        //retval = target_read_u16(target, (target_addr_t)(wa_start + i), &reg16);
        reg16 = *(wa_start+i);
        target_write_u16(target, FACI_CMD_AREA, reg16);
        do
        {
            retval = target_read_u32(target, FACI_FSTATR, &reg32);
            usleep(1000);
        } while(reg32 & 0X400);
    }

    target_write_u8(target, FACI_CMD_AREA, 0XD0);

    retval = target_read_u32(target, FACI_FSTATR, &reg32);
    if (reg32 & 0X8000)
    {
        retval = target_read_u8(target, FACI_FASTAT, &reg8);
    }

	return retval;
}

static int rv40f_write_block(struct flash_bank *bank, const uint8_t *buffer,
        uint32_t offset, uint32_t count)
{
    struct renesas_bank *info = bank->driver_priv;
    struct target *target = bank->target;
    uint32_t buffer_size = 2048;		/* Default minimum value */
    struct working_area *write_algorithm;
    struct working_area *source;
    uint32_t address = bank->base + offset;
    struct reg_param reg_params[4];
    __attribute__ ((unused)) struct armv7m_algorithm armv7m_info;
    int retval = ERROR_OK;
    unsigned thisrun_count;
    uint8_t r8, reg8;
    //int i = 0;

    /* Increase buffer_size if needed */
    if (buffer_size < (target->working_area_size / 2))
        buffer_size = (target->working_area_size / 2);


    /* RAMCODE used for rv40f Flash programming:                 */
    /* R0 keeps number of halfwords to write (4,8,16 - data, 64 - program) */
    /* R1 keeps source start address         (u32SourceStart)       */
    /* R2 keeps source end address           (u32SourceEnd)       */
    /* R3 keeps target start address         (u32Target)       */

    static const uint8_t rv40f_flash_write_code[] = {
#if 1
/* Autogenerated with ../../../src/helper/bin2char.sh */
0x00,0x48,0x00,0xbe,0x00,0xe0,0x7f,0x40,
    };
#else
/* Autogenerated with ../../../src/helper/bin2char.sh */
0x89,0x46,0x4c,0x46,0x0e,0x4d,0x0f,0x4f,0x81,0x46,0x4e,0x46,0x2b,0x63,0xe8,0x20,
0x38,0x70,0x48,0x46,0x38,0x70,0x20,0x88,0x38,0x80,0x02,0x34,0x01,0x3e,0x09,0xd0,
0x20,0x88,0x38,0x80,0x02,0x34,0x80,0x20,0x40,0x19,0x01,0x68,0x06,0x48,0x08,0x42,
0xf9,0xd1,0xf3,0xe7,0xd0,0x20,0x38,0x70,0x80,0x21,0x49,0x19,0x03,0x48,0x00,0xbe,
0x00,0xe0,0x7f,0x40,0x00,0x00,0x7e,0x40,0x00,0x04,0x00,0x00,0x78,0x56,0x34,0x12};
        0x89, 0x46, 0x4c, 0x46, 0x0e, 0x4d, 0x0f, 0x4f,
        0x81, 0x46, 0x4e, 0x46, 0x2b, 0x63, 0xe8, 0x20,
        0x38, 0x70, 0x48, 0x46, 0x38, 0x70, 0x20, 0x88,
        0x38, 0x80, 0x02, 0x34, 0x01, 0x3e, 0x09, 0xd0,
        0x20, 0x88, 0x38, 0x80, 0x02, 0x34, 0x80, 0x20,
        0x40, 0x19, 0x01, 0x68, 0x06, 0x48, 0x08, 0x42,
        0xf9, 0xd1, 0xf3, 0xe7, 0xd0, 0x20, 0x38, 0x70,
        0x80, 0x21, 0x49, 0x19, 0x08, 0x68, 0x00, 0xbe,
        0x00, 0xe0, 0x7f, 0x40, 0x00, 0x00, 0x7e, 0x40,
        0x00, 0x00, 0x00, 0x00
    };
#endif

    LOG_INFO("Renesas RV40F FLASH Write ...");

    if(info->is_data){
    /* check alignment - data flash */
        if (offset & 0x7) {
            LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
            return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
        }
    }else{
        /* check alignment - code flash */
        if (offset & 0x7f) {
            LOG_WARNING("offset 0x%" PRIx32 " breaks required 128-byte alignment", offset);
            return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
        }
    }

    retval = target_read_u8(target, FACI_FASTAT, &reg8);

    /* enter P/E mode */
    retval = rv40f_pe_mode(bank);

    retval = target_read_u8(target, FACI_FASTAT, &reg8);

    if (retval != ERROR_OK)
        return retval;

    retval = target_write_u32(target, 0X407FE014, 0x00);

    if (retval != ERROR_OK)
    {
        LOG_ERROR("Failed disable interrupt");
        return retval;
    }

    retval = target_write_u32(target, 0X407FE078, 0x01);

    if (retval != ERROR_OK)
    {
        LOG_ERROR("Failed disable block protection");
        return retval;
    }

    retval = target_read_u8(target, FACI_FASTAT, &reg8);
    count = count / 2;		/* number bytes -> number halfwords */

    /* allocate working area and variables with flash programming code */
    if (target_alloc_working_area(target, sizeof(rv40f_flash_write_code),
	    &write_algorithm) != ERROR_OK) {
        LOG_WARNING("no working area available, can't do block memory writes");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    retval = target_write_buffer(target, write_algorithm->address,
        sizeof(rv40f_flash_write_code), rv40f_flash_write_code);
    if (retval != ERROR_OK)
        return retval;

    /* memory buffer */
    while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
        buffer_size /= 2;
        if (buffer_size <= 256) {
        /* free working area, write algorithm already allocated */
            target_free_working_area(target, write_algorithm);

            LOG_WARNING("No large enough working area available, can't do block memory writes");
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
       }
   }

   armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
   armv7m_info.core_mode = ARM_MODE_THREAD;

   init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* number of halfwords to program */
   init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* source start address */
   init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* source end address */
   init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /* target start address */

   /* write code buffer and use Flash programming code within rv40f           */
   /* Set breakpoint to 0 with time-out of 1000 ms                          */
   while (count > 0) {

        /* dataflash has a write count 2,4,8 halfwords */
        if(info->is_data){
            if(count >= 8){
                thisrun_count = 8;
            }else if(count >= 4){
                thisrun_count = 4;
            }else{
                thisrun_count = 2;
            }
        /* code flash has 64 halfwords programming block */
        }else{
            thisrun_count = 64;
        }

        //for (i = 0; i < 16; i++)
            //LOG_INFO("buffer[%d]@%lx=%hhx", i, source->address + i*2, (uint8_t)buffer[i]);

        /* 好像没有真正写进去 */
        retval = target_write_buffer(target, source->address, thisrun_count * 2, buffer);
        if (retval != ERROR_OK)
            break;

        buf_set_u32(reg_params[0].value, 0, 32, thisrun_count);
        buf_set_u32(reg_params[1].value, 0, 32, source->address);
        buf_set_u32(reg_params[2].value, 0, 32, source->address + thisrun_count * 2);
        buf_set_u32(reg_params[3].value, 0, 32, address);
        //LOG_INFO("%d src:[%x,%x] dst:%x", thisrun_count, (unsigned int)source->address, (unsigned int)source->address + thisrun_count*2, address);

        //retval = target_read_u8(target, FACI_FASTAT, &reg8);
#if 1
        /* 执行这个算法的时候出错了 */
        retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
            write_algorithm->address, 0, 1000, &armv7m_info);
#else
        //__rv40f_flash_write(target, thisrun_count, (uint16_t *)source->address, (uint16_t *)source->address + thisrun_count, address);
        __rv40f_flash_write(target, thisrun_count, (uint16_t *)buffer, (uint16_t *)source->address + thisrun_count, address);
#endif
        //LOG_INFO("user code ans=%d", retval);
        retval = target_read_u8(target, FACI_FASTAT, &reg8);

        if (retval != ERROR_OK) {
            LOG_ERROR("Error executing RV40f Flash programming algorithm");
            retval = ERROR_FLASH_OPERATION_FAILED;
            break;
        }

        target_read_u8(bank->target, FACI_FWEPROR, &r8);

        if(retval != ERROR_OK || r8 != 1){
            LOG_ERROR("Flash locked for write");
            retval = ERROR_FLASH_OPERATION_FAILED;
            break;
        }

        /* wait for write 1000 ms */
        retval = rv40f_complete(target, 1000);

        if (retval != ERROR_OK)
            return retval;

        /* decrement counters */
        if(count >= thisrun_count){
            buffer  += thisrun_count * 2;
            address += thisrun_count * 2;
            count   -= thisrun_count;
        }else{
            count = 0;
            break;
        }
        //LOG_USER("count=%u", count);
    }

    /* switch back to readonly after operation */
    retval = rv40f_to_readonly(target);

    target_free_working_area(target, source);
    target_free_working_area(target, write_algorithm);

    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);
    destroy_reg_param(&reg_params[2]);
    destroy_reg_param(&reg_params[3]);

    return retval;
}

static int rv40f_probe(struct flash_bank *bank)
{
    struct renesas_bank *info = bank->driver_priv;
    struct flash_sector *s;
    unsigned n, num_pages;

    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }
    info->probed = false;

    bank->base = info->base;
    bank->size = info->size;		/* bytes */

    /* data flash - same for all types */
    if(info->base > 0){
        info->is_data = 1;
        num_pages = info->size/64; /* 1 block = 64 bytes */
        bank->num_sectors = num_pages;
        bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
        for(n = 0; n < num_pages; n++){
            s = &bank->sectors[n];
            s->offset = n*64;
            s->size = 64;
            s->is_erased = -1;
            s->is_protected = -1;
        }
    /* code flash 8k x 8 + 32k x N */
    }else{
        info->is_data = 0;
        num_pages = (info->size - 8*8192) / 32768 + 8;
        bank->num_sectors = num_pages;
        bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
        for(n = 0; n < 8; n++){
            s = &bank->sectors[n];
            s->offset = n*8192;
            s->size = 8192;
            s->is_erased = -1;
            s->is_protected = -1;
        }
        /* rest of sectors (bigger) */
        for(; n < num_pages; n++){
            s = &bank->sectors[n];
            s->offset = (n - 8)*32768 + 8*8192;
            s->size = 32768;
            s->is_erased = -1;
            s->is_protected = -1;
        }
    }

    info->probed = true;

    return ERROR_OK;
}

static int rv40f_auto_probe(struct flash_bank *bank)
{
    struct renesas_bank *info = bank->driver_priv;
    if (info->probed)
        return ERROR_OK;
    return rv40f_probe (bank);
}

/* this is the public interface to this module. 'renesas_rv40f_flash' is added to the list of
 * available flash drivers in 'drivers.c'
 */
const struct flash_driver renesas_rv40f_flash = {
    .name = "renesas_rv40f",
    /* .commands = ??, */
    .flash_bank_command = rv40f_flash_bank_command,
    .erase = rv40f_erase,
    /* .protect = ??, */
    .write = rv40f_write_block,
    .read = default_flash_read,
    .probe = rv40f_probe,
    .auto_probe = rv40f_auto_probe,
    .erase_check = default_flash_blank_check,
    /* .protect_check = ??, */
    /* .info = ??, */
    .free_driver_priv = default_flash_free_driver_priv,
    .usage = "flash bank bank_id 'renesas_rv40f' base_address size"
};

