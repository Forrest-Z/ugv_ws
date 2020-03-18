//
// Created by nvidia on 1/23/19.
//

#ifndef TX2_CAN_CAN_INTERFACE_HPP
#define TX2_CAN_CAN_INTERFACE_HPP

/*
 * lib.h - library include for command line tools
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
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
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#ifndef CAN_UTILS_LIB_H
#define CAN_UTILS_LIB_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>


#define can_device "can0"


/* reset to default */

#define ATTRESET "\33[0m"

/* attributes */

#define ATTBOLD      "\33[1m"
#define ATTUNDERLINE "\33[4m"
#define ATTBLINK     "\33[5m"
#define ATTINVERSE   "\33[7m"
#define ATTINVISIBLE "\33[8m"

/* foreground colors */

#define FGBLACK   "\33[30m"
#define FGRED     "\33[31m"
#define FGGREEN   "\33[32m"
#define FGYELLOW  "\33[33m"
#define FGBLUE    "\33[34m"
#define FGMAGENTA "\33[35m"
#define FGCYAN    "\33[36m"
#define FGWHITE   "\33[37m"

/* background colors */

#define BGBLACK   "\33[40m"
#define BGRED     "\33[41m"
#define BGGREEN   "\33[42m"
#define BGYELLOW  "\33[43m"
#define BGBLUE    "\33[44m"
#define BGMAGENTA "\33[45m"
#define BGCYAN    "\33[46m"
#define BGWHITE   "\33[47m"

/* cursor */

#define CSR_HOME  "\33[H"
#define CSR_UP    "\33[A"
#define CSR_DOWN  "\33[B"
#define CSR_RIGHT "\33[C"
#define CSR_LEFT  "\33[D"

#define CSR_HIDE  "\33[?25l"
#define CSR_SHOW  "\33[?25h"

/* clear screen */

#define CLR_SCREEN  "\33[2J"

/* for hardware timestamps - since Linux 2.6.30 */
#ifndef SO_TIMESTAMPING
#define SO_TIMESTAMPING 37
#endif

/* from #include <linux/net_tstamp.h> - since Linux 2.6.30 */
#define SOF_TIMESTAMPING_SOFTWARE (1<<4)
#define SOF_TIMESTAMPING_RX_SOFTWARE (1<<3)
#define SOF_TIMESTAMPING_RAW_HARDWARE (1<<6)

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */ //即最大借口数为16 就是最多是can16
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */ //接收如何canbus的数据
#define ANL "\r\n"    /* newline in ASC mode */                                 //回车换行

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON  2  /* silent mode (completely silent) */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

/* buffer sizes for CAN frame string representations */

#define CL_ID (sizeof("12345678##1"))
#define CL_DATA sizeof(".AA")
#define CL_BINDATA sizeof(".10101010")

 /* CAN FD ASCII hex short representation with DATA_SEPERATORs */
#define CL_CFSZ (2*CL_ID + 64*CL_DATA)

/* CAN FD ASCII hex long representation with binary output */
#define CL_LONGCFSZ (2*CL_ID + sizeof("   [255]  ") + (64*CL_BINDATA))

/* CAN DLC to real data length conversion helpers especially for CAN FD */

/* get data length from can_dlc with sanitized can_dlc */
unsigned char can_dlc2len(unsigned char can_dlc);

/* map the sanitized data length to an appropriate data length code */
unsigned char can_len2dlc(unsigned char len);

unsigned char asc2nibble(char c);
/*
 * Returns the decimal value of a given ASCII hex character.
 *
 * While 0..9, a..f, A..F are valid ASCII hex characters.
 * On invalid characters the value 16 is returned for error handling.
 */

int hexstring2data(char *arg, unsigned char *data, int maxdlen);
/*
 * Converts a given ASCII hex string to a (binary) byte string.
 *
 * A valid ASCII hex string consists of an even number of up to 16 chars.
 * Leading zeros '00' in the ASCII hex string are interpreted.
 *
 * Examples:
 *
 * "1234"   => data[0] = 0x12, data[1] = 0x34
 * "001234" => data[0] = 0x00, data[1] = 0x12, data[2] = 0x34
 *
 * Return values:
 * 0 = success
 * 1 = error (in length or the given characters are no ASCII hex characters)
 *
 * Remark: The not written data[] elements are initialized with zero.
 *
 */

int parse_canframe(char *cs, struct canfd_frame *cf);
/*
 * Transfers a valid ASCII string decribing a CAN frame into struct canfd_frame.
 *
 * CAN 2.0 frames
 * - string layout <can_id>#{R{len}|data}
 * - {data} has 0 to 8 hex-values that can (optionally) be separated by '.'
 * - {len} can take values from 0 to 8 and can be omitted if zero
 * - return value on successful parsing: CAN_MTU
 *
 * CAN FD frames
 * - string layout <can_id>##<flags>{data}
 * - <flags> a single ASCII Hex value (0 .. F) which defines canfd_frame.flags
 * - {data} has 0 to 64 hex-values that can (optionally) be separated by '.'
 * - return value on successful parsing: CANFD_MTU
 *
 * Return value on detected problems: 0
 *
 * <can_id> can have 3 (standard frame format) or 8 (extended frame format)
 * hexadecimal chars
 *
 *
 * Examples:
 *
 * 123# -> standard CAN-Id = 0x123, len = 0
 * 12345678# -> extended CAN-Id = 0x12345678, len = 0
 * 123#R -> standard CAN-Id = 0x123, len = 0, RTR-frame
 * 123#R0 -> standard CAN-Id = 0x123, len = 0, RTR-frame
 * 123#R7 -> standard CAN-Id = 0x123, len = 7, RTR-frame
 * 7A1#r -> standard CAN-Id = 0x7A1, len = 0, RTR-frame
 *
 * 123#00 -> standard CAN-Id = 0x123, len = 1, data[0] = 0x00
 * 123#11 22 33 44 55 66 77 88 -> standard CAN-Id = 0x123, len = 8
 * 123#11.22.33.44.55.66.77.88 -> standard CAN-Id = 0x123, len = 8
 * 123#11.2233.44556677.88 -> standard CAN-Id = 0x123, len = 8
 * 32345678#112233 -> error frame with CAN_ERR_FLAG (0x2000000) set
 *
 * 123##0112233 -> CAN FD frame standard CAN-Id = 0x123, flags = 0, len = 3
 * 123##1112233 -> CAN FD frame, flags = CANFD_BRS, len = 3
 * 123##2112233 -> CAN FD frame, flags = CANFD_ESI, len = 3
 * 123##3 -> CAN FD frame, flags = (CANFD_ESI | CANFD_BRS), len = 0
 *     ^^
 *     CAN FD extension to handle the canfd_frame.flags content
 *
 * Simple facts on this compact ASCII CAN frame representation:
 *
 * - 3 digits: standard frame format
 * - 8 digits: extendend frame format OR error frame
 * - 8 digits with CAN_ERR_FLAG (0x2000000) set: error frame
 * - an error frame is never a RTR frame
 * - CAN FD frames do not have a RTR bit
 */

void fprint_canframe(FILE *stream , struct canfd_frame *cf, char *eol, int sep, int maxdlen);
void sprint_canframe(char *buf , struct canfd_frame *cf, int sep, int maxdlen);
/*
 * Creates a CAN frame hexadecimal output in compact format.
 * The CAN data[] is separated by '.' when sep != 0.
 *
 * The type of the CAN frame (CAN 2.0 / CAN FD) is specified by maxdlen:
 * maxdlen = 8 -> CAN2.0 frame
 * maxdlen = 64 -> CAN FD frame
 *
 * 12345678#112233 -> extended CAN-Id = 0x12345678, len = 3, data, sep = 0
 * 12345678#R -> extended CAN-Id = 0x12345678, RTR, len = 0
 * 12345678#R5 -> extended CAN-Id = 0x12345678, RTR, len = 5
 * 123#11.22.33.44.55.66.77.88 -> standard CAN-Id = 0x123, dlc = 8, sep = 1
 * 32345678#112233 -> error frame with CAN_ERR_FLAG (0x2000000) set
 * 123##0112233 -> CAN FD frame standard CAN-Id = 0x123, flags = 0, len = 3
 * 123##2112233 -> CAN FD frame, flags = CANFD_ESI, len = 3
 *
 * Examples:
 *
 * fprint_canframe(stdout, &frame, "\n", 0); // with eol to STDOUT
 * fprint_canframe(stderr, &frame, NULL, 0); // no eol to STDERR
 *
 */

#define CANLIB_VIEW_ASCII	0x1
#define CANLIB_VIEW_BINARY	0x2
#define CANLIB_VIEW_SWAP	0x4
#define CANLIB_VIEW_ERROR	0x8
#define CANLIB_VIEW_INDENT_SFF	0x10

#define SWAP_DELIMITER '`'

void fprint_long_canframe(FILE *stream , struct canfd_frame *cf, char *eol, int view, int maxdlen);
void sprint_long_canframe(char *buf , struct canfd_frame *cf, int view, int maxdlen);
/*
 * Creates a CAN frame hexadecimal output in user readable format.
 *
 * The type of the CAN frame (CAN 2.0 / CAN FD) is specified by maxdlen:
 * maxdlen = 8 -> CAN2.0 frame
 * maxdlen = 64 -> CAN FD frame
 *
 * 12345678   [3]  11 22 33 -> extended CAN-Id = 0x12345678, dlc = 3, data
 * 12345678   [0]  remote request -> extended CAN-Id = 0x12345678, RTR
 * 14B0DC51   [8]  4A 94 E8 2A EC 58 55 62   'J..*.XUb' -> (with ASCII output)
 * 20001111   [7]  C6 23 7B 32 69 98 3C      ERRORFRAME -> (CAN_ERR_FLAG set)
 * 12345678  [03]  11 22 33 -> CAN FD with extended CAN-Id = 0x12345678, dlc = 3
 *
 * 123   [3]  11 22 33         -> CANLIB_VIEW_INDENT_SFF == 0
 *      123   [3]  11 22 33    -> CANLIB_VIEW_INDENT_SFF == set
 *
 * Examples:
 *
 * // CAN FD frame with eol to STDOUT
 * fprint_long_canframe(stdout, &frame, "\n", 0, CANFD_MAX_DLEN);
 *
 * // CAN 2.0 frame without eol to STDERR
 * fprint_long_canframe(stderr, &frame, NULL, 0, CAN_MAX_DLEN);
 *
 */

void snprintf_can_error_frame(char *buf, size_t len, const struct canfd_frame *cf,
                  const char *sep);
/*
 * Creates a CAN error frame output in user readable format.
 */

#endif
int candump(int argc, char **argv,void * operatePtr);
int cansend(int argc, char **argv);
void * readData(void * operatePtr);
void * writeData(char * can_data);
void * Callback_Read(struct canfd_frame frame,void * operatePtr);
char get_char_X16(int data);
int get_int_X16(char data);
void executeCMD(const char *cmd, char *result);
int create_and_set_can();
char *decimal_to_binary(int n);

#endif //TX2_CAN_CAN_INTERFACE_HPP
