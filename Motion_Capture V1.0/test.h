#ifndef	 _TEST_H
#define  _TEST_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_common_define.h"
#include "mc_init_8051.h"

void a_conversion(long temp_data);
void a_display_x();
void a_display_y();
void a_display_z();
void DisplayOneChar(uchar out);

void i_conversion(uchar *s,int temp_data);
void i_display_x();
void i_display_y();
void i_display_z();
void DisplayListChar(uchar *DData,L);

void h_conversion(uint temp_data);
void h_display_dierection();
void h_display_x();
void h_display_y();
void h_display_z();

extern uchar data_write[18];

#endif /* _TEST_H */