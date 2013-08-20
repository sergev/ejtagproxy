/*
 * Generic debug interface to a target microcontroller.
 *
 * Copyright (C) 2011-2012 Serge Vakulenko
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. The name of the author may not be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
typedef struct _target_t target_t;

target_t *target_open (void);
void target_close (target_t *t, int power_on);

unsigned target_idcode (target_t *t);
const char *target_cpu_name (target_t *t);
int target_is_rom_address (target_t *t, unsigned addr);

unsigned target_read_word (target_t *t, unsigned addr);
void target_read_block (target_t *t, unsigned addr,
	unsigned nwords, unsigned *data);

void target_write_word (target_t *t, unsigned addr, unsigned word);
void target_write_block (target_t *t, unsigned addr,
	unsigned nwords, unsigned *data);
void target_cache_flush (target_t *t, unsigned addr);

void target_stop (target_t *t);
void target_step (target_t *t);
void target_resume (target_t *t);
void target_run (target_t *t, unsigned addr);
void target_restart (target_t *t);
int target_is_stopped (target_t *t, int *is_aborted);

unsigned target_read_register (target_t *t, unsigned regno);
unsigned target_read_cop0_register (target_t *t, unsigned regno, unsigned sel);
void target_write_register (target_t *t, unsigned regno, unsigned val);

void target_add_break (target_t *t, unsigned addr, int type);
void target_remove_break (target_t *t, unsigned addr);
