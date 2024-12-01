# ProyectoArqui

# ALU Encoding Table

| **ALUControl (4 bits)** | **Operación** | **Descripción**            | **Fórmula (Expresión de ALUResult)**         |
|-------------------------|---------------|----------------------------|---------------------------------------------|
| 0000                    | ADD           | Suma de SrcA y SrcB         | ALUResult = SrcA + SrcB                     |
| 0001                    | SUB           | Resta de SrcA y SrcB        | ALUResult = SrcA - SrcB                     |
| 0010                    | MOV           | Mover SrcB a ALUResult      | ALUResult = SrcB                           |
| 0011                    | AND           | AND bit a bit de SrcA y SrcB | ALUResult = SrcA & SrcB                     |
| 0100                    | ORR           | OR bit a bit de SrcA y SrcB  | ALUResult = SrcA | SrcB                     |
| 0101                    | MLS           | Multiplicación y resta      | ALUResult = SrcC - (SrcA * SrcB)            |
| 0110                    | EOR           | XOR bit a bit de SrcA y SrcB | ALUResult = SrcA ^ SrcB                     |
| 0111                    | MVN           | Negar SrcB                  | ALUResult = ~SrcB                           |
| 1000                    | MLA           | Multiplicación y suma       | ALUResult = SrcC + (SrcA * SrcB)            |
| 1001                    | RSB           | Resta invertida             | ALUResult = SrcB - SrcA                     |
| 1010                    | ADC           | Suma con CarryIn            | ALUResult = SrcA + SrcB + CarryIn           |
| 1011                    | SBC           | Resta con CarryIn           | ALUResult = SrcA - SrcB - ~CarryIn          |
| 1100                    | MUL           | Multiplicación              | ALUResult = SrcA * SrcB                     |
