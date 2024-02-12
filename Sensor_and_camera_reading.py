import serial

# Imposta la porta seriale e la velocità di trasmissione (baud rate)
# modificare la stringa '/dev/tty.usbserial-140' con quella presente nella configurazione finale
# /dev/ttyUSB0 è quella in alto a destra, vista dal raspberry
ser = serial.Serial('/dev/tty.usbserial-140',
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