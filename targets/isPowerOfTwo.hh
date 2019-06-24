/* sematrope - superoptimizer using the z3 SMT solver
   Copyright (C) 2019  Falk Hüffner

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef SEMATROPE_ISPOWEROFTWO_HH
#define SEMATROPE_ISPOWEROFTWO_HH

#include "sematrope.hh"

// Test whether a nonzero number is a power of two.
__attribute__((constructor)) void registerIsPowerOfTwoNonzero() {
    constexpr auto REGISTER_WIDTH = 8;

    Target target = {
	.name = "isPowerOfTwoNonzero",
	.registerWidth = REGISTER_WIDTH,
	.arity = 1,
	.target = [](const std::vector<z3::expr>& xs) {
	    assert(xs.size() == 1);
	    const auto& x = xs[0];
	    auto r = context.bool_val(false);
	    uint64_t p = 1;
	    for (int i = 0; i < REGISTER_WIDTH; ++i) {
		r = r || (x == bvConst(p));
		p <<= 1;
	    }
	    return boolToBv(r);
	},
	.isValidInput = [](const std::vector<z3::expr>& xs) {
	    assert(xs.size() == 1);
	    const auto& x = xs[0];
	    return x != 0;
	},
    };

    registerTarget(target);
}

#endif  // SEMATROPE_ISPOWEROFTWO_HH
