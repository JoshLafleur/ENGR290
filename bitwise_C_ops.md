`[...]`

--> That being said, an alternative way to set/clear specific bits without affecting others may be useful to consider using

## SETTING: BITWISE OR + BITMASK

- e.g. suppose we have a registered called `REG` with contents `0000 0101`, and we want to set the 5th bit to 1 (so it becomes `0001 0101`). Note: in these examples I assume `REG` returns the contents of a register. In actual C++ code you usually have to dereference a pointer to get the contents.

- we can achieve this by doing `REG = (REG | (0001 0000))`, where on the RHS of the bitwise OR operator we have our bit mask. So the RHS of the assignment operator resolves to `0001 0101`, and is then stored in `REG`.

- a nice shorthand way of doing the above example in C++ looks like:
  - `REG |= (1 << 4); // bit mask shifts "1" 4 times to the right so that we target the 5th bit (i.e bit number 4)`

## CLEARING: BITWISE AND + INVERTED BITMASK

- e.g. we want to clear the 5th bit of `REG` (which now has `0001 0101` stored)

- we can achieve this with `REG = (REG & ~(0001 0000))` i.e. `REG = (REG & (1110 1111)`. Only the 0 in this bit mask will force whatever bit it gets ANDed with to become 0. The 1s of the bitmask do not change any corresponding bits.

- shorthand: `REG &= ~(1 << 4);`

`[...]`

--> In this lab you have to check certain bits are set or not. To do this you can check against hex values, which is fine, though forces us to perhaps check bits we don't need to. We can employ bitmasks to simplify the process and check only specific bits we are interested in.

## CHECKING IF 5TH BIT IS UNSET: BITMASK + AND

- `if (REG & (1 << 4) == 0)` will do the trick, since all the 0's in the bitmask will force 0's on the bits in REG, the only way we get 0 in the end is if the bit ANDed with our "1" from the bit mask meets a 0.

## CHECKING IF 5TH BIT IS SET: BITMASK + AND

- `if (REG & (1 << 4) == 1)` will do the trick, since the only way we get a resulting 1 is if the 5th bit in REG is 1, just like in our bit mask.
- This can be further shortened down to `if (REG & (1 << 4))`, since "1" is considered true in C++, and all other numerical values are considered false.
