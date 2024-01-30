/*
    ** file: asm.S
    */
#include <xc.h>
    .text
    .set noreorder 
/*********************************************************************
* appFunction()
* This function jump specified address
********************************************************************/
.global appFunction
.ent appFunction
appFunction:
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    jals 0x9D005000
    nop
    
.end appFunction     
    