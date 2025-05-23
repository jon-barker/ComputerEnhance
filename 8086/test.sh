FNAME=listing_0037_single_register_mov
nasm ~/data/computer_enhance/perfaware/part1/${FNAME}.asm -o ${FNAME}
./sim8086 ${FNAME} > ${FNAME}.asm
diff <(grep -v '^;' ~/data/computer_enhance/perfaware/part1/${FNAME}.asm) <(grep -v '^;' ${FNAME}.asm)

FNAME=listing_0038_many_register_mov
nasm ~/data/computer_enhance/perfaware/part1/${FNAME}.asm -o ${FNAME}
./sim8086 ${FNAME} > ${FNAME}.asm
diff <(grep -v '^;' ~/data/computer_enhance/perfaware/part1/${FNAME}.asm) <(grep -v '^;' ${FNAME}.asm)

FNAME=listing_0039_more_movs
nasm ~/data/computer_enhance/perfaware/part1/${FNAME}.asm -o ${FNAME}
./sim8086 ${FNAME} > ${FNAME}.asm
diff <(grep -v '^;' ~/data/computer_enhance/perfaware/part1/${FNAME}.asm) <(grep -v '^;' ${FNAME}.asm)

FNAME=listing_0040_challenge_movs
nasm ~/data/computer_enhance/perfaware/part1/${FNAME}.asm -o ${FNAME}
./sim8086 ${FNAME} > ${FNAME}.asm
diff <(grep -v '^;' ~/data/computer_enhance/perfaware/part1/${FNAME}.asm) <(grep -v '^;' ${FNAME}.asm)
