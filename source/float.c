#include "float.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// This function is useful on systems without float print support.
// Explodes f to int and frac parts. Fraction part is put as a string to *frac_str.
void float_explode(double f, int *int_part, char *frac_str, size_t frac_str_size) {
	double fac;
	double fract_part_double;
	unsigned int fract_part;
	size_t printed;
	char s_frac[frac_str_size];
	int i;
	int pad_left_needed;

	fac = pow(10, frac_str_size);
	*int_part = (int)f;

	if (f < 0)
		f *= -1;

	fract_part_double = f-(double)(int)f;
	// Getting the fractional part, one more digit than we need.
	fract_part = (int)(fract_part_double*(fac+1));

	// Round last digit.
	fract_part += 5;

	// Cut the last, unneeded digit.
	fract_part /= 10;

	fract_part_double = fract_part * (1/fac);

	printed = snprintf(s_frac, frac_str_size, "%u", fract_part);

	// Left padding the fractional part with zeroes.
	pad_left_needed = (frac_str_size-1)-printed;
	for (i = 0; i < pad_left_needed; i++)
		frac_str[i] = '0';
	memcpy(frac_str+i, s_frac, printed);
	frac_str[frac_str_size-1] = 0;
}
