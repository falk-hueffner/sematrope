/* sematrope - superoptimizer using the z3 SMT solver
   Copyright (C) 2018  Falk Hüffner

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

#ifndef SEMATROPE_SEMATROPE_HH
#define SEMATROPE_SEMATROPE_HH

#include <string>
#include <functional>

#include <z3++.h>

struct Target {
    std::string name;
    int registerWidth = 0;
    int arity = 1;
    std::function<z3::expr(const std::vector<z3::expr>&)> target;
    std::function<z3::expr(const std::vector<z3::expr>&)> isValidInput;
};

extern z3::context context;
extern int registerWidth;
extern int shiftMask;

z3::expr bvConst(uint64_t x);
void registerTarget(const Target& target);
z3::expr boolToBv(const z3::expr& b);

#endif  // SEMATROPE_SEMATROPE_HH
