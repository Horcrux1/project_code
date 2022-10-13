# Copyright: Pouyan B. Navard @ Photogrammetric Computervision Labaratory 
# Reference: Enhancing Photographs with Near Infrared Images, Zhang et al. 2008
from skimage.exposure import match_histograms
import matplotlib.pyplot as plt
import numpy as np
import cv2
import pywt

class EnhanceImage():
	def __init__(self, RGB_IMG_DIR, NIR_IMG_DIR):

		rgb_img = cv2.imread(RGB_IMG_DIR) # the order of the channel is BGR
		nir_image = cv2.imread(NIR_IMG_DIR)

		# Convert RBB to HSV color space 
		H, S, V = self.rgb2hsv(rgb_img)

		# compute weighted region mask 
		p_s = self.compute_prob(S)
		p_v = self.compute_prob(V)
		W = self.compute_weighted_region_mask(S, V, p_s, p_v)

		# compute Haar wavelength transform for the value and NIR image\
		V, VH, VV, VD = self.Haar_wavelength_transform(V)
		N, NH, NV, ND = self.Haar_wavelength_transform(nir_image)

		# large scale contrast transfer
		V_prime = self.large_scale_contrast_transfer(V, N, W)

		# transfer texture 
		VH_prime, VV_prime, VD_prime = self.transfer_texture(VH, VV, VD, NH, NV, ND, W)

		enhanced_V = self.render_enhanced_visible_channel(V_prime, VH_prime, VV_prime, VD_prime)

		self.BGR_IMG = self.render_enhanced_BGR(H, S, enhanced_V)

	def rgb2hsv(self, rgb_image):
		# convert RGB channel to HSV 
		hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV) 
		H, S, V = cv2.split(hsv)
		return H, S, V

	def hsv2bgr(self, hsv_image):
		# convert HSV channel to RGB
		rgb = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR) 
		return rgb


	def compute_prob(self, channel, num_bin=255, show=False):
		# compute the probability of the pixel intensity based on the histogram 
		n, bins, patches = plt.hist(channel.flatten(), bins=num_bin, density=True)
		# compute probability here
		prob = None
		return prob

	def compute_weighted_region_mask(self, S, V, P_S, P_V):
		"""
		Compute weighted region mask using W_s and W_V

			S: Saturation channel in HSV color space
			V: Value channel in HSV color space 
			P_S: probability of the pixel in the saturation channel 
			P_V: probability of the pixel in the value channel

		"""
		W_s = np.ones(S.shape()) - np.exp(-P_s @ np.absolute(S - 1) )
		W_v = np.ones(V.shape()) - np.exp(-P_v @ np.absolute(v - 0.5))
		# element wise multiplication 
		W = np.multiply(W_s, W_v)
		# Gaussian blur to remove the noise 
		W = cv2.GaussianBlur(W, (5,5), 0)
		return W

	def Haar_wavelength_transform(self, signal)
		# Haar decomposition. Approximation (cA), Horizontal(cH), Vertical(cV), Diagonal(cD) 
		cA, (cH, cV, cD) = pywt.dwt2(signal, 'haar')
		return cA, cH, cV, cD

	def large_scale_contrast_transfer(self, V, N, W):
		"""
		Transfer the large scale transfer and form a new visible band (v_prime)
			V: Approximation value in the Haar transform applied on the visible band (V in HSV)
			N: Approximation value in the Haar transform applied on the NIR image 
			W: Weighted region mask 

		"""
		# Apply bilateral filtering to decompose images to large-scale layer and detail layer, and use the larger-scale layer as brightness map
		VL = cv2.bilateralFilter(V, 15, 80, 80, Core.BORDER_DEFAULT)
		VD = V - VL
		NL = cv2.bilateralFilter(N, 15, 80, 80, Core.BORDER_DEFAULT)
		ND = N - NL
		
		# perform histogram matching between VL (source) and NL (destination)
		VL_prime = match_histograms(VL, NL)

		resize_W = cv2.resize(W, VL_prime.shape())
		V_prime = np.multiply(resize_W, VL_prime + VD) + np.multiply(1 - resize_W, V)

		return V_prime


	def transfer_texture(self, VH, VV, VD, NH, NV, ND, W):
		"""
			VH: Horizontal detail in Haar transform applied on V
			VV: Vetical detail in Haar transform applied on V
			VD: Diagonal detail in Haar transform applied on V
			NH: Horizontal detail in Haar transform applied on NIR image
			NV: Vertical detail in Haar transform applied on NIR image
			ND: Diagonal detail in Haar transform applied on NIR image
			W: Weighted region mask 
		"""
		resize_W = cv2.resize(W, VH.shape())

		VH_prime = np.multiply(resize_W, NH) + np.multiply(1 - resize_W, VH)
		VV_prime = np.multiply(resize_W, NV) + np.multiply(1 - resize_W, VV)
		VD_prime = np.multiply(resize_W, ND) + np.multiply(1 - resize_W, VD)

		return VH_prime, VV_prime, VD_prime

	def render_enhanced_visible_channel(self, V_prime, VH_prime, VV_prime, VD_prime):
		"""
		Applying inverse Haar transform to enhance the V-channel using the enhanced elements with better contrast and texture 
			V_prime: Enhanced approximation value of Haar transform 
			VH_prime: Enhanced horizontal value of Haar transform
			VV_prime: Enhanced vertical value of Haar transform
		 	VD_prime: Enhanced diagonal value of Haar transform
		"""
		coeff = (V_prime, (VH_prime, VV_prime, VD_prime))
		enhanced_V = pywt.idwt2(coeff, 'haar')

		return enhanced_V

	def render_enhanced_BGR(self, H, S, enhanced_V):
		"""
		Rerender the image in HSV color space with the enhanced visible channel and convert it back to BGR space (compatible to opencv imread and imwrite)
		H: Hue 
		S: Saturation
		enhanced_V: enhanced version of the value channel
		"""

		# concantante H, S and enhanced_V along the third axis
		HSV = np.dstack((H, S, enhanced_V))
		bgr = hsv2bgr(HSV)

		return bgr

	def save_enhanced_image(file_name):
		cv2.imwrite(file_name, self.BGR_IMG)

if __name__ == "__main__":

	enhancer = EnhanceImage("RGB_IMG.jpg", "NIR_IMG.jpg")
	enhancer.save_enhanced_image("./enhanced_IMG.png")
