import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import math
import heapq

farba_prah = 5
krok_mriezky = 10
maximalna_sirka = 900
maximalna_vyska = 900
#definícia triedy so základnými widgetmi a vlastnosťami
class GPSNavigator(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.grid(row=0, column=0, sticky="nsew")
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)
        self.grid_columnconfigure(0, weight=1)

        self.obrazok = None
        self.cesta = None
        self.vzdialenost = None

        self.canvas = tk.Canvas(self, bg="white", borderwidth=2, relief="sunken")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.spodne_menu = tk.Frame(self, borderwidth=0)
        self.spodne_menu.grid(row=1, column=0, sticky="ew")

        self.mierka_frame = tk.Frame(self, borderwidth=0)
        self.mierka_frame.grid(row=2, column=0, sticky="ew")

        self.mierka_popis = tk.Label(self.mierka_frame, text="Mierka (1 cm : skutočná dĺžka v metroch):")
        self.mierka_popis.pack(side="left")
        self.mierka_entry = tk.Entry(self.mierka_frame)
        self.mierka_entry.pack(side="left")

        self.rychlost_frame = tk.Frame(self, borderwidth=0)
        self.rychlost_frame.grid(row=3, column=0, sticky="ew")
        self.rychlost_popis = tk.Label(self.rychlost_frame, text="Rýchlosť pohybu robota (m/s):")
        self.rychlost_popis.pack(side="left")
        self.rychlost_entry = tk.Entry(self.rychlost_frame)
        self.rychlost_entry.pack(side="left")
        
        self.vypocet_frame = tk.Frame(self, borderwidth=0)
        self.vypocet_frame.grid(row=4, column=0, sticky="ew")
        self.vypocet_button = tk.Button(self.vypocet_frame, text="Vypočítať najmenší čas", command=self.vypocitaj_cas)
        self.vypocet_button.pack(side="left")
        
        self.cas_entry = tk.Entry(self.vypocet_frame, state="readonly")
        self.cas_entry.pack(side="left", padx=10)
        self.cas_label = tk.Label(self.vypocet_frame, text="(s)")
        self.cas_label.pack(side="left")
        
        self.reset_button = tk.Button(self.spodne_menu, text="Resetovať výber", command=self.resetuj)
        self.reset_button.pack(side="top", pady=5)
        
        self.ppi_label = tk.Label(self.spodne_menu, text="PPI (hustota pixelov na 1 palec):")
        self.ppi_label.pack(side="left")
        
        self.ppi_entry = tk.Entry(self.spodne_menu)
        self.ppi_entry.pack(side="left")
        
        self.limit = 0
        self.tk_obrazok = None
        self.siva = None
        self.vrcholy = []
        self.zaciatok = None
        self.koniec = None
        self.graf = {}
        self.canvas.bind("<Button-1>", self.platno_klik)
        master.title("Optimalizácia riadenia pohybu robota")

        menubar = tk.Menu(master)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Nahrať mapu", command=self.nacitaj_mapu)
        filemenu.add_separator()
        filemenu.add_command(label="Východ", command=master.quit)
        menubar.add_cascade(label="File", menu=filemenu)
        master.config(menu=menubar)
#funkcia na načítanie výkresu výrobnej haly
    def nacitaj_mapu(self):
        cesta_suboru = filedialog.askopenfilename(filetypes=[("Image Files", "*.jpeg;*.jpg;*.png")])
        if not cesta_suboru:
            return
        self.canvas.delete("all")
        self.obrazok = Image.open(cesta_suboru)

        if self.obrazok.width > maximalna_sirka or self.obrazok.height > maximalna_vyska:
            messagebox.showwarning("Upozornenie", "Obrázok je príliš veľký.")
            return

        self.tk_obrazok = ImageTk.PhotoImage(self.obrazok)
        self.siva = self.obrazok.convert("L")
        self.canvas.configure(width=self.obrazok.width, height=self.obrazok.height)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_obrazok)
#funkcia je určená na vymazanie všetkých charakteristík mapy
    def resetuj(self):
        self.zaciatok = None
        self.koniec = None
        self.vrcholy = []
        self.graf = {}
        self.canvas.delete("start")
        self.canvas.delete("konec")
        self.canvas.delete("edges")
        self.canvas.delete("vertex")
        self.canvas.delete("cesta")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_obrazok)
#funkcia je určená na výber vrcholov v rovine mapy
    def platno_klik(self, event):
        if event.x>self.obrazok.width or event.x <=0 or event.y<=0 or event.y>=self.obrazok.height:
            return 
        if self.obrazok is None:
            return
        if self.siva.getpixel((event.x, event.y)) <= farba_prah:
            return
        
        if self.zaciatok is None:
            self.zaciatok = (event.x, event.y)
            self.canvas.create_oval(event.x-5, event.y-5, event.x+5, event.y+5, outline="blue", fill="blue", width=2, tag="start")
        elif self.koniec is None:
            self.koniec = (event.x, event.y)
            if self.zaciatok == self.koniec:
                return
            self.canvas.create_oval(event.x-5, event.y-5, event.x+5, event.y+5, outline="red", fill="red", width=2, tag="konec")
            self.vygeneruj_vrcholy()
            self.vytvor_hrany()
            self.cesta, self.vzdialenost = self.dijkstrov_algoritmus()
            if self.cesta:
                self.zobraz_cestu(self.cesta)
            else:
                messagebox.showinfo("Informácia", "Cesta nebola nájdená.")
#funkcia je určená na generovanie vrcholov
    def vygeneruj_vrcholy(self):
        x_min = max(min(self.zaciatok[0], self.koniec[0]) - self.limit, 0)
        x_max = min(max(self.zaciatok[0], self.koniec[0]) + self.limit, self.obrazok.width)
        y_min = max(min(self.zaciatok[1], self.koniec[1]) - self.limit, 0)
        y_max = min(max(self.zaciatok[1], self.koniec[1]) + self.limit, self.obrazok.height)
        pixely = self.siva.load()
        self.vrcholy = []

        for y in range(y_min, y_max, krok_mriezky):
            for x in range(x_min, x_max, krok_mriezky):
                if pixely[x, y] > farba_prah:
                    self.vrcholy.append((x, y))

        for bod in [self.zaciatok, self.koniec]:
            if bod not in self.vrcholy:
                self.vrcholy.append(bod)
#funkcia je určená na overenie platnosti hrany
    def je_hrana_platna(self, a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        kroky = int(max(abs(dx), abs(dy)))
        if kroky == 0:
            return True
        for i in range(kroky + 1):
            t = i / kroky
            x = round(a[0] + dx * t)
            y = round(a[1] + dy * t)
            if self.siva.getpixel((x, y)) <= farba_prah:
                return False
        return True
#funkcia je určená na vytvorenie hrán 
    def vytvor_hrany(self):
        n = len(self.vrcholy)
        self.graf = {i: [] for i in range(n)}
        for i in range(n):
            for j in range(i+1, n):
                if self.je_hrana_platna(self.vrcholy[i], self.vrcholy[j]):
                    vzd = math.hypot(self.vrcholy[i][0] - self.vrcholy[j][0],
                                     self.vrcholy[i][1] - self.vrcholy[j][1])
                    self.graf[i].append((j, vzd))
                    self.graf[j].append((i, vzd))
#implementácia Dijkstrovho algoritmu
    def dijkstrov_algoritmus(self):
        try:
            start_idx = self.vrcholy.index(self.zaciatok)
        except ValueError:
            self.vrcholy.append(self.zaciatok)
            start_idx = self.vrcholy.index(self.zaciatok)
        try:
            koniec_idx = self.vrcholy.index(self.koniec)
        except ValueError:
            self.vrcholy.append(self.koniec)
            koniec_idx = self.vrcholy.index(self.koniec)
        n = len(self.vrcholy)
        vzdialenost = [float("inf")] * n
        vzdialenost[start_idx] = 0
        predchodca = [None] * n
        halda = [(0, start_idx)]
        while halda:
            d, u = heapq.heappop(halda)
            if u == koniec_idx:
                break
            for v, w in self.graf.get(u, []):
                x = d + w
                if x < vzdialenost[v]:
                    vzdialenost[v] = x
                    predchodca[v] = u
                    heapq.heappush(halda, (x, v))
        if vzdialenost[koniec_idx] == float("inf"):
            self.limit += 10
            self.vrcholy = []
            self.vygeneruj_vrcholy()
            self.vytvor_hrany()
            return self.dijkstrov_algoritmus()
        cesta = []
        u = koniec_idx
        while u is not None:
            cesta.append(u)
            u = predchodca[u]
        cesta.reverse()
        return cesta, vzdialenost[koniec_idx]
#funkcia je určená na výpočet najmenšieho času
    def vypocitaj_cas(self):
        self.cas_entry.config(state="normal")
        if not self.ppi_entry.get() or not self.mierka_entry.get() or not self.rychlost_entry.get() or self.koniec is None:
            messagebox.showwarning("POZOR", "Musíte vyplniť všetky vstupné polia.")
            return
        ppi = int(self.ppi_entry.get())
        dlzka_pixela = 2.54 / ppi
        mierka = int(self.mierka_entry.get())
        rychlost = float(self.rychlost_entry.get())
        skutocna_dlzka = self.vzdialenost * mierka * dlzka_pixela
        cas = skutocna_dlzka / rychlost
        self.cas_entry.delete(0, 'end')
        self.cas_entry.insert(0, str(round(cas, 3)))
        self.cas_entry.config(state="readonly")
#funkcia je určená na zobrazenie cesty
    def zobraz_cestu(self, cesta):
        self.canvas.delete("cesta")
        for i in range(len(cesta) - 1):
            x1, y1 = self.vrcholy[cesta[i]]
            x2, y2 = self.vrcholy[cesta[i + 1]]
            self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2, tag="cesta")
#hlavný program
if __name__ == "__main__":
    root = tk.Tk()
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    app = GPSNavigator(root)
    root.mainloop()