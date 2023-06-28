# Author: Zhenhui YU

## Project Description：
This is the project integrated BSSRDF importance sampling for rendering translucent objects.  
This work referenced "BSSRDF Importance Sampling" proposed by  Alan King et al.  
The slide can be found at "https://www.solidangle.com/research/s2013_bssrdf_slides.pdf".  
The paper can be found at "https://dl.acm.org/doi/10.1145/2504459.2504520".  
The reflectance profiles referenced "Approximate Reflectance Profiles for Efficient Subsurface Scattering" proposed by Per H. Christensen and Brent Burley.  
The paper can be found at "https://graphics.pixar.com/library/ApproxBSSRDF/index.html".  

## Features:
•	Integrated BSSRDF importance sampling into the code of path tracing to render translucent objects.  
•	Calculated the diffusion profile and probability density function for evaluating the weights of sample points.  
•	Projected sample rays along the three axes of the local shading frame to avoid high-variance results.  

## Notes:
I ran this code in the Linux system.  
It Is implemented for learning purposes.  
