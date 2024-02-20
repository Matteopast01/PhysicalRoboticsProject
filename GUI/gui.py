import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Controllo di Start e Stop")
        self.master.geometry("850x600")
        self.master.resizable(False, False)

        # Carica e ruota l'immagine
        image = Image.open("robot.jpg")
        image = image.rotate(180)
        image = image.resize((800, 400), Image.ANTIALIAS)
        photo = ImageTk.PhotoImage(image)
        self.label = tk.Label(master, image=photo)
        self.label.image = photo  # Evita la garbage collection
        self.label.pack(pady=20)

        # Pulsanti
        self.stop_button = tk.Button(master, text="Stop", command=self.stop, font=("Helvetica", 20), bg="#f44336", fg="white", padx=30, pady=15, state=tk.DISABLED)
        self.stop_button.pack(side=tk.BOTTOM, pady=20)

        self.start_button = tk.Button(master, text="Start", command=self.start, font=("Helvetica", 20), bg="#4CAF50", fg="white", padx=30, pady=15)
        self.start_button.pack(side=tk.BOTTOM)

    def start(self):
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        messagebox.showinfo("Messaggio", "La simulazione si avvierà tra pochi istanti!")

    def stop(self):
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        messagebox.showinfo("Messaggio", "La simulazione è stata interrotta!")

def main():
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
