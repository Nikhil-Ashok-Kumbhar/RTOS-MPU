 ;*****************************************************************************

 ;*****************************************************************************

 ;
 ;*****************************************************************************

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

 .def initSp
 .def setPsp
 .def getPsp
 .def pushR4_11
 .def popR4_11
 .def pushDummy
 .def getR0
 .def getR1
 .def setUnpriv
 .def getSp
;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;------------ -----------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

initSp:
			  MOV    R4, #2
              MSR    CONTROL, R4
              BX     LR
setPsp:
               MSR    PSP, R0
               BX     LR                     ; return from subroutine

getPsp:
			   MRS    R0,PSP
			   BX     LR                     ; return from subroutine

pushR4_11:
			   MRS    R0,PSP
			   STMDB R0!, {R4-R11}
			   MSR PSP, R0
			   BX     LR                     ; return from subroutine

popR4_11:
			   LDMIA R0!, {R4-R11}
			   MSR PSP, R0
			   BX     LR                     ; return from subroutine

pushDummy:
			  MOV    R1,R0
			  MRS    R0,PSP
			  MOV    R2,#0x01000000
			  SUB    R0, R0,#4
			  STR    R2,[R0]
			  SUB    R0, R0,#4
			  STR    R1,[R0]
			  SUB    R0,R0,#20
			  STR    R0,[R0]
			  MSR    PSP,R0
			  BX    LR


getR0:

			   BX     LR                     ; return from subroutine
getR1:
			   MOV   R0,R1
			   BX     LR                     ; return from subroutine

setUnpriv:
			  MOV    R4, #3
              MSR    CONTROL, R4
              ISB
              BX     LR                   ; return from subroutine


getSp:       MOV    R0, SP
			 BX     LR                   ; return from subroutine


.endm
