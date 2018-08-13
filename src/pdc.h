/*
 *  PDC FB kernel driver
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef PDC_H_
#define PDC_H_

void pdc_config(void *pdc_addr, int *dims, unsigned int *fbPA);
void pdc_start(void *pdc_addr);
void pdc_stop(void *pdc_addr);

#define PDC_REG_FBADDR 0x0068
#define PDC_REG_SWAP 0x0078
#define PDC_REG_STATUS 0x007C

#endif // PDC_H_
