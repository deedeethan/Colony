/*
 * Copyright (c) 2009, Kohsuke Ohtani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * busio.h - BUS i/o operations
 */

#ifndef _ARM_BUSIO_H
#define _ARM_BUSIO_H

#include <sys/cdefs.h>
#include <sys/types.h>

/* temp!! */
#define IOBASE	0x80000000

#define bus_read_8(addr)	(*((volatile uint8_t *)(IOBASE+addr)))
#define bus_read_16(addr)	(*((volatile uint16_t *)(IOBASE+addr)))
#define bus_read_32(addr)	(*((volatile uint32_t *)(IOBASE+addr)))

#define bus_write_8(addr, val)	(*((volatile uint8_t *)(IOBASE+addr)) = (val))
#define bus_write_16(addr, val)	(*((volatile uint16_t *)(IOBASE+addr)) = (val))
#define bus_write_32(addr, val)	(*((volatile uint32_t *)(IOBASE+addr)) = (val))

#endif /* !_ARM_BUSIO_H */
