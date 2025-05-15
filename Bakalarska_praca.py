import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import math
import heapq


PAINT = 5
STEP = 10
MAX_WIDTH = 600
MAX_HEIGHT = 300


class GPS_Navigator(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.grid(row=0, column=0, sticky="nsew")
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)
        self.grid_columnconfigure(0, weight=1)

        self.image = None
        self.gray = None
        self.tkimage = None
        self.path = None
        self.scale_factor = 1.0
        self.dist = None
        self.limit = 0

        self.vertices = []
        self.start_vertex = None
        self.end_vertex = None
        self.graph = {}


        self.canvas = tk.Canvas(self, bg="white", borderwidth=2, relief="sunken")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Button-1>", self.on_canvas_click)


        self.bottom_frame = tk.Frame(self)
        self.bottom_frame.grid(row=1, column=0, sticky="ew")

        self.mierka_frame = tk.Frame(self)
        self.mierka_frame.grid(row=2, column=0, sticky="ew")

        self.mierka1 = tk.Frame(self)
        self.mierka1.grid(row=3, column=0, sticky="ew")

        self.rychlost_frame = tk.Frame(self)
        self.rychlost_frame.grid(row=4, column=0, sticky="ew")

        self.vypocet = tk.Frame(self)
        self.vypocet.grid(row=5, column=0, sticky="ew")


        menubar = tk.Menu(root)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Nahrať mapu", command=self.load_map)
        filemenu.add_separator()
        filemenu.add_command(label="Východ", command=root.quit)
        menubar.add_cascade(label="File", menu=filemenu)
        root.config(menu=menubar)


        self.reset_ = tk.Button(self.bottom_frame, text="Resetovať výber", command=self.reset)
        self.reset_.pack(side="top", pady=5)

        self.ppi_label = tk.Label(self.bottom_frame, text="PPI (hustota pixelov na 1 palec):")
        self.ppi_label.pack(side="left")
        self.PPI = tk.Entry(self.bottom_frame)
        self.PPI.pack(side="left", padx=0)

        self.mierka_label = tk.Label(self.mierka1, text="Mierka (1 cm : skutočná dĺžka v metroch):")
        self.mierka_label.pack(side="left")
        self.mierka2_entry = tk.Entry(self.mierka1)
        self.mierka2_entry.pack(side="left")

        self.rychlost_label = tk.Label(self.rychlost_frame, text="Rýchlosť pohybu robota (m/s):")
        self.rychlost_label.pack(side="left")
        self.rychlost_entry = tk.Entry(self.rychlost_frame)
        self.rychlost_entry.pack(side="left")

        self.vypocet1 = tk.Button(self.vypocet, text="Vypočítať najmenší čas", command=self.spracovanie_tlacidla)
        self.vypocet1.pack(side="left")

        self.najkartsi_cas = tk.Entry(self.vypocet, state="readonly")
        self.najkartsi_cas.pack(side="left", padx=10)

        self.sec = tk.Label(self.vypocet, text="(s)")
        self.sec.pack(side="left")

        master.title("Optimalizácia riadenia pohybu robota")

    def load_map(self):
        filepath = filedialog.askopenfilename(filetypes=[("Image Files", "*.jpeg;*.jpg;*.png")])
        if not filepath:
            return
        self.canvas.delete("all")
        self.image = Image.open(filepath)
        self.tkimage = ImageTk.PhotoImage(self.image)
        self.gray = self.image.convert("L")
        self.canvas.configure(width=self.image.width, height=self.image.height)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tkimage)

    def reset(self):
        self.start_vertex = None
        self.end_vertex = None
        self.vertices = []
        self.graph = {}
        self.canvas.delete("start")
        self.canvas.delete("end")
        self.canvas.delete("edges")
        self.canvas.delete("vertex")
        self.canvas.delete("path")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tkimage)

    def on_canvas_click(self, event):
        if self.image is None or self.gray.getpixel((event.x, event.y)) <= PAINT:
            return

        if self.start_vertex is None:
            self.start_vertex = (event.x, event.y)
            self.canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5,
                                    outline="blue", fill="blue", width=2, tag="start")
        elif self.end_vertex is None:
            self.end_vertex = (event.x, event.y)
            if self.start_vertex == self.end_vertex:
                return
            self.canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5,
                                    outline="red", fill="red", width=2, tag="end")
            self.generate_vertices()
            self.build_graph_edges()
            self.path, self.dist = self.dijkstras_algorithm()
            if self.path:
                self.highlight_path(self.path)
            else:
                messagebox.showinfo("Informácia", "Cesta nebola nájdená")

    def generate_vertices(self):
        x_coords = [self.start_vertex[0], self.end_vertex[0]]
        y_coords = [self.start_vertex[1], self.end_vertex[1]]
        x_min = max(min(x_coords) - self.limit, 0)
        y_min = max(min(y_coords) - self.limit, 0)
        x_max = min(max(x_coords) + self.limit, self.image.width)
        y_max = min(max(y_coords) + self.limit, self.image.height)

        pixels = self.gray.load()
        self.vertices = []

        for y in range(y_min, y_max, STEP):
            for x in range(x_min, x_max, STEP):
                if pixels[x, y] > PAINT:
                    self.vertices.append((x, y))

        for pt in [self.start_vertex, self.end_vertex]:
            if pt not in self.vertices:
                self.vertices.append(pt)

    def is_valid_edge(self, v1, v2):
        dx = v2[0] - v1[0]
        dy = v2[1] - v1[1]
        steps = int(max(abs(dx), abs(dy)))
        if steps == 0:
            return True
        for i in range(steps + 1):
            t = i / steps
            x = round(v1[0] + dx * t)
            y = round(v1[1] + dy * t)
            if self.gray.getpixel((x, y)) <= PAINT:
                return False
        return True

    def build_graph_edges(self):
        n = len(self.vertices)
        self.graph = {i: [] for i in range(n)}
        for i in range(n):
            for j in range(i + 1, n):
                if self.is_valid_edge(self.vertices[i], self.vertices[j]):
                    dist = math.hypot(
                        self.vertices[i][0] - self.vertices[j][0],
                        self.vertices[i][1] - self.vertices[j][1]
                    )
                    self.graph[i].append((j, dist))
                    self.graph[j].append((i, dist))

    def dijkstras_algorithm(self):
        start_index = self.vertices.index(self.start_vertex)
        end_index = self.vertices.index(self.end_vertex)

        n = len(self.vertices)
        self.dist = [float('inf')] * n
        self.dist[start_index] = 0
        cesta = [None] * n
        halda = [(0, start_index)]

        while halda:
            d, u = heapq.heappop(halda)
            if u == end_index:
                break
            for v, w in self.graph.get(u, []):
                x = d + w
                if x < self.dist[v]:
                    self.dist[v] = x
                    cesta[v] = u
                    heapq.heappush(halda, (x, v))

        if self.dist[end_index] == float('inf'):
            self.limit += 10
            self.vertices = []
            self.generate_vertices()
            self.build_graph_edges()
            return self.dijkstras_algorithm()

        path = []
        u = end_index
        while u is not None:
            path.append(u)
            u = cesta[u]
        path.reverse()

        return path, self.dist[end_index]

    def spracovanie_tlacidla(self):
        self.najkartsi_cas.config(state="normal")
        if not self.PPI.get() or not self.mierka2_entry.get() or not self.rychlost_entry.get() or self.end_vertex is None:
            messagebox.showwarning("POZOR", "Na začiatku musíte vyplniť všetky informačné polia.")
            return

        ppi = int(self.PPI.get())
        mierka = float(self.mierka2_entry.get())
        rychlost = float(self.rychlost_entry.get())
        dlzka_pixela = 2.54 / ppi
        dlzka_cesty = self.dist * mierka * dlzka_pixela
        cas = dlzka_cesty / rychlost

        self.najkartsi_cas.delete(0, 'end')
        self.najkartsi_cas.insert(0, str(round(cas, 3)))
        self.najkartsi_cas.config(state="readonly")

    def highlight_path(self, path):
        self.canvas.delete("path")
        for i in range(len(path) - 1):
            x1, y1 = self.vertices[path[i]]
            x2, y2 = self.vertices[path[i + 1]]
            self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2, tag="path")


if __name__ == "__main__":
    root = tk.Tk()
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    app = GPS_Navigator(root)
    root.mainloop()