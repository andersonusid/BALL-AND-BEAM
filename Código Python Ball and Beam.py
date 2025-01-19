import tkinter as tk
from tkinter import ttk
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Configuração Serial
ser = serial.Serial('COM3', 115200, timeout=1)

# Variáveis globais
dados_x = []
dados_y = []
kp = ki = kd = None

def ler_dados_serial():
    global kp, ki, kd
    if ser.in_waiting > 0:
        linha = ser.readline().decode('utf-8').strip()
        if linha.startswith("Posicao:"):
            posicao = float(linha.split(":")[1])
            dados_x.append(len(dados_x) * 0.06)  # Intervalo de 60ms
            dados_y.append(posicao)
            atualizar_grafico()
        elif "Kp:" in linha:
            kp = float(linha.split("Kp:")[1])
        elif "Ki:" in linha:
            ki = float(linha.split("Ki:")[1])
        elif "Kd:" in linha:
            kd = float(linha.split("Kd:")[1])
            atualizar_parametros()
        elif linha.startswith("Nova referencia:"):
            ref_label.config(text=f"Ref: {linha.split(':')[1]}")  # Atualiza o valor da referência na interface
    root.after(10, ler_dados_serial)

def atualizar_grafico():
    ax.clear()
    ax.plot(dados_x, dados_y, label="Posição")
    ax.set_xlim(0, max(dados_x) if dados_x else 10)
    ax.set_ylim(0, 500)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Posição")
    ax.legend()
    canvas.draw()

def atualizar_parametros():
    kp_label.config(text=f"Kp: {kp}")
    ki_label.config(text=f"Ki: {ki}")
    kd_label.config(text=f"Kd: {kd}")

def enviar_ref():
    try:
        novo_ref = float(ref_entry.get())  # Obtém o valor digitado na caixa de texto
        comando = f"SET_REF:{novo_ref}\n"
        ser.write(comando.encode())  # Envia o comando para o Arduino
        ref_label.config(text=f"Ref: {novo_ref}")  # Atualiza a exibição do valor de referência
    except ValueError:
        ref_label.config(text="Erro: valor inválido")

def reiniciar_autotune():
    ser.write(b"REINICIAR_AUTOTUNE\n")
    dados_x.clear()
    dados_y.clear()
    ax.clear()
    canvas.draw()

# Configuração da interface Tkinter
root = tk.Tk()
root.title("Controle PID - Sistema Barra e Bola")

# Gráfico
fig, ax = plt.subplots(figsize=(5, 4))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Parâmetros PID
param_frame = ttk.Frame(root)
param_frame.pack(side=tk.LEFT, padx=10, pady=10)
ttk.Label(param_frame, text="Parâmetros PID").pack()

kp_label = ttk.Label(param_frame, text="Kp: N/A")
kp_label.pack()
ki_label = ttk.Label(param_frame, text="Ki: N/A")
ki_label.pack()
kd_label = ttk.Label(param_frame, text="Kd: N/A")
kd_label.pack()

# Alteração de Referência
ref_frame = ttk.Frame(root)
ref_frame.pack(side=tk.LEFT, padx=10, pady=10)

ttk.Label(ref_frame, text="Alterar Referência").pack()
ref_entry = ttk.Entry(ref_frame)
ref_entry.pack(pady=5)
ref_button = ttk.Button(ref_frame, text="Enviar", command=enviar_ref)
ref_button.pack(pady=5)
ref_label = ttk.Label(ref_frame, text="Ref: N/A")
ref_label.pack()

# Botões
button_frame = ttk.Frame(root)
button_frame.pack(side=tk.RIGHT, padx=10, pady=10)

clear_button = ttk.Button(button_frame, text="Clear", command=lambda: [dados_x.clear(), dados_y.clear(), atualizar_grafico()])
clear_button.pack(pady=5)

reiniciar_button = ttk.Button(button_frame, text="Reiniciar Autotune", command=reiniciar_autotune)
reiniciar_button.pack(pady=5)

# Loop de leitura e interface
ler_dados_serial()
root.mainloop()
