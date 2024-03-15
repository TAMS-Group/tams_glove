.section .init
.global main
# li gp, 0x10000000
jal ra, main
ebreak
