#include "printf_float.h"
#include "debug.h"

void Print_Float_Value(float value, uint32 scale)
{
    uint32 integer;
    uint32 fraction;
    uint32 decimal_places;
    float abs_value;

    if (value < 0.0f)
    {
        mcu_printf("-");
        abs_value = -value;
    }
    else
    {
        abs_value = value;
    }

    if (abs_value > 0.0f && abs_value < 1e-9f) {
        int exponent = 0;
        float mantissa = abs_value;
        
        while (mantissa < 1.0f && exponent > -15) {
            mantissa *= 10.0f;
            exponent--;
        }
        
        mcu_printf("%d.", (int)mantissa);
        uint32 frac = (uint32)((mantissa - (int)mantissa) * 100.0f);
        mcu_printf("%02de%d", frac, exponent);
        return;
    }

    integer  = (uint32)abs_value;
    
    float frac_part = (abs_value - (float)integer) * (float)scale;
    fraction = (uint32)(frac_part + 0.5f);
    
    if (fraction >= scale) {
        integer++;
        fraction = 0;
    }

    if (scale == 1000000000) decimal_places = 9;
    else if (scale == 1000000) decimal_places = 6;
    else if (scale == 1000) decimal_places = 3;
    else if (scale == 100) decimal_places = 2;
    else if (scale == 10) decimal_places = 1;
    else decimal_places = 0;

    if (decimal_places == 9)
        mcu_printf("%d.%09d", integer, fraction);
    else if (decimal_places == 6)
        mcu_printf("%d.%06d", integer, fraction);
    else if (decimal_places == 3)
        mcu_printf("%d.%03d", integer, fraction);
    else if (decimal_places == 2)
        mcu_printf("%d.%02d", integer, fraction);
    else if (decimal_places == 1)
        mcu_printf("%d.%01d", integer, fraction);
    else
        mcu_printf("%d", integer);
}