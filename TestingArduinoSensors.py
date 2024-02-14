import serial

# Imposta la porta seriale e la velocità di trasmissione (baud rate)
# modificare la stringa con quella presente nella configurazione finale

SERIAL_PORT = '/dev/tty.usbserial-140'      # mac di Giacomo
# SERIAL_PORT = '/dev/ttyUSB0'              # Raspberry

ser = serial.Serial(SERIAL_PORT,
                    9600)

try:
    while True:
        # Leggi una riga dalla seriale
        line = ser.readline().decode('utf-8').strip()
        # Stampa la riga letta
        print(line)

except KeyboardInterrupt:
    # Chiudi la porta seriale in caso di interruzione manuale (Ctrl+C)
    ser.close()
    print("Chiusura della connessione seriale.")
