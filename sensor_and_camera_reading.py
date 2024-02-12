import serial

# Imposta la porta seriale e la velocità di trasmissione (baud rate)
# modificare la stringa con quella presente nella configurazione finale
# '/dev/tty.usbserial-140' è quella del mac di Giacomo
# '/dev/ttyUSB0' è quella del Raspberry in alto a destra
ser = serial.Serial('/dev/ttyUSB0',
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