I2C - SEND DATA TO SLAVE
- generate a start bit
- send 7 bit address + 1 wite bit
- check ACK bit from slave
- send REG ADDR slave
- send WRITE DATA
- check ACK
- generate stop bit

I2C - READ DATA FROM SLAVE
- generate a start bit
- send 7 bit address + 1 WRITE bit
- check ACK bit from slave
- send REG ADDR slave
- check ACK bit from slave
- generate a Re - start bit
- send 7 bit address + 1 READ bit (HIGH - 1)
- check ACK bit from slave
- READ DATA from slave
- generate a stop bit
