#!/usr/bin/python

# You need to run:
# sudo pip install pymodbus

from pymodbus.client.sync import ModbusSerialClient as ModbusClient

modbus_timeout = .1 # NOTE! This is default 3 seconds. It waits for the timeout before returning I think. Making this .1 works well. The pyepsolar project uses a 1 second timeout.

client = ModbusClient(method = 'rtu', port = 'COM1', baudrate = 115200, timeout=1)
client.connect()

result = client.read_input_registers(0x3100,6,unit=1)
solarVoltage = float(result.registers[0] / 100.0)
solarCurrent = float(result.registers[1] / 100.0)
batteryVoltage = float(result.registers[4] / 100.0)
chargeCurrent = float(result.registers[5] / 100.0)

print solarVoltage
print solarCurrent
print batteryVoltage
print chargeCurrent
 
client.close()
