from math import *
import matplotlib.pyplot as plt
import numpy as np
import sys

# blue update
# orange prediction
# La funzione `f` calcola la densità di probabilità di una distribuzione gaussiana 
# data una media (`mu`), una varianza (`sigma2`) e un valore di input `x`.
def f(mu, sigma2, x):
	# Calcolo del coefficiente della distribuzione gaussiana
	coefficient = 1.0 / sqrt(2.0 * pi * sigma2)  
	# Calcolo dell'esponenziale della distribuzione gaussiana
	exponential = exp(-0.5 * (x - mu) ** 2 / sigma2)  
	# Restituisce la densità di probabilità per il valore `x`
	return coefficient * exponential  


# La funzione `plot` disegna due distribuzioni gaussiane: 
# una per l'aggiornamento ("update") e una per la previsione ("predict").
def plot(mu_update, sig_update, mu_predict, sig_predict):
	# Definisce un intervallo di valori x per il grafico
	x_axis = np.arange(-5, 25, 0.1)

	# Crea una lista dei valori della gaussiana per l'aggiornamento
	g_update = []
	for x in x_axis:
		g_update.append(f(mu_update, sig_update, x))

	# Crea una lista dei valori della gaussiana per la previsione
	g_predict = []
	for x in x_axis:
		g_predict.append(f(mu_predict, sig_predict, x))

	# Traccia entrambe le gaussiane nel grafico
	plt.plot(x_axis, g_update)  # Grafico della gaussiana di aggiornamento (blu)
	plt.plot(x_axis, g_predict)  # Grafico della gaussiana di previsione (arancione)
	plt.show()  # Mostra il grafico

# La funzione `update` implementa la fase di aggiornamento del filtro di Kalman.
# Combina due gaussiane (una che rappresenta la stima corrente e una che rappresenta la misurazione)
# restituendo una nuova media e varianza.
def update(mean1, var1, mean2, var2):
	# Calcola la nuova media come una combinazione pesata delle due gaussiane
	new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
	# Calcola la nuova varianza combinando inversamente le due varianze
	new_var = 1 / (1 / var1 + 1 / var2)
	return [new_mean, new_var]  # Restituisce la nuova media e varianza

# print update(10.,4.,12.,4.)


# La funzione `predict` implementa la fase di previsione del filtro di Kalman.
# Aggiorna la stima basandosi su un'azione di controllo o movimento.
def predict(mean1, var1, mean2, var2, U):
	# Calcola la nuova media sommando la stima precedente, il movimento (`mean2`) e un controllo `U`
	new_mean = mean1 + mean2 + U
	# Calcola la nuova varianza sommando le incertezze (varianze)
	new_var = var1 + var2
	return [new_mean, new_var]  # Restituisce la nuova media e varianza


if __name__ == "__main__":
	# measurements è una lista di valori misurati:
	# - ogni elemento rappresenta un'osservazione presa da un sensore o da un'altra fonte.
	# - questi valori saranno usati nella fase di aggiornamento (update) per correggere la stima.
	measurements: list = [3.0, 4.0, 6.0, 7.0, 8.0]

	# motion è una lista di valori di movimento o controllo: 
	# - ogni elemento rappresenta il movimento effettuato tra due misurazioni successive
	# - questi valori saranno usati nella fase di previsione (predict)
	motion: list = [1.0, 2.0, 1.0, 1.0, 1.0]

	# measurement_sigma rappresenta la deviazione standard delle misurazioni:
	# rappresenta l'incertezza associata ai dati delle misurazioni.
	measurement_sigma: float = 4.0

	# Questo parametro indica l'incertezza nel movimento:
	# un valore alto implica che il modello di movimento è meno affidabile.
	# Sarà usato nella fase di previsione.
	motion_sigma: float = 2.0

	# mu rappresenta la stima iniziale della posizione. 
	# All'inizio del filtro di Kalman, non si ha una stima precisa della posizione, 
	# quindi viene inizializzata a 0.
	mu: float = 0.0

	# position_sigma rappresenta la deviazione standard iniziale della posizione.
	# All'inizio, l'incertezza sulla posizione è molto grande (10000), 
	# indicando che non si ha fiducia nella stima iniziale (mu = 0).
	# Verrà aggiornato man mano che il filtro elabora le misurazioni.
	position_sigma: int = 10000.0
	
	U: float = 0.0

	# Ciclo per aggiornare e prevedere
	for n in range(len(measurements)):
		# Fase di aggiornamento
		mu, position_sigma = update(mu, position_sigma, measurements[n], measurement_sigma)
		print(f"Update step {n+1}: [mean, var] = {mu, position_sigma}")
		
		# Salva per il grafico
		mu_u = mu
		position_sigma_u = position_sigma

		# Fase di previsione
		mu, position_sigma = predict(mu, position_sigma, motion[n], motion_sigma, U)
		print(f"Predict step {n+1}: [mean, var] = {mu, position_sigma}")

		# Traccia il risultato
		plot(mu_u, position_sigma_u, mu, position_sigma)