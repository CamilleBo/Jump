// MODELE GYMNAST

/*
Q1 = Translation antérieure
Q2 = Translation verticale
Q3 = Rotation du bassin vers l'arrière
Q4 = Extension du thorax
Q5 = Extension de la tête
Q6 = Flexion à l'épaule droite
Q7 = Adduction à l'épaule droite
Q8 = Flexion au coude droit
Q9 = Flexion à l'épaule gauche
Q10 = Abduction à l'épaule gauche
Q11 = Flexion au coude gauche
Q12 = Flexion à la hanche droite
Q13 = Extension au genou droit
Q14 = Dorsiflexion du pied droit
Q15 = Flexion à la hanche gauche
Q16 = Extension au genou gauche
Q17 = Dorsiflexion du pied gauche
*/

version	2

// General informations
root_actuated 0
external_forces 0

// Informations about Right foot segment 
	segment	FootR
		translations 	xz
		rotations	y
		mass	0.70991
		RT
			1.00000	0.00000	0.00000	0.000
			0.00000	1.00000	0.00000	0.000
			0.00000	0.00000	1.00000	0.000
			0.00000	0.00000	0.00000	1.0000
		inertia
			0.00160	0.00000	0.00000
			0.00000	0.00178	0.00000
			0.00000	0.00000	0.00051
                com	0.00000	0.00000	0.17450-0.06997
                mesh 	0	0	0.17450
                mesh 	0	0	0
                mesh 	-0.037	0	0.0250+0.17450
                mesh 	0	0	0.17450
	endsegment


// Informations about Right leg segment
	segment	LegR
		parent FootR
		rotations	y
		mass	3.20698
		RT
			1.00000	0.00000	0.00000	0.000
			0.00000	1.00000	0.00000	0.000
                        0.00000	0.00000	1.00000	0.17450
			0.00000	0.00000	0.00000	1.0000
		inertia
			0.03877	0.00000	0.00000
			0.00000	0.03877	0.00000
			0.00000	0.00000	0.00408
		com	0.00000	0.00000	0.39050-0.16793
		mesh 	0	0	0
		mesh	0	0	0.39050
	endsegment


// Informations about Cuisses segment
	segment	ThighR
		parent LegR
		RT
			1.00000	0.00000	0.00000	0.000
			0.00000	1.00000	0.00000	0.000
			0.00000	0.00000	1.00000	0.39050
			0.00000	0.00000	0.00000	1.00000
		rotations	y
		mass	6.47744
		inertia
			0.07275	0.00000	0.00000
			0.00000	0.07275	0.00000
			0.00000	0.00000	0.01915
		com	0.00000	0.00000 0.35350-0.15110
		mesh 	0	0	0
		mesh	0	0	0.35350
	endsegment


	


// Informations about Pelvis segment
	segment	Pelvis
		parent ThighR
		RT
			1.00000	0.00000	0.00000	0.00000
                        0.00000	1.00000	0.00000	0.082
			0.00000	0.00000	1.00000 0.35350
			0.00000	0.00000	0.00000	1.00000
		rotations	y
		mass	8.89519
		inertia
			0.04592	0.00000	0.00000
			0.00000	0.07587	0.00000
			0.00000	0.00000	0.06693
		com	0.00000	0.00000 0.09222
		mesh	0	0	0.20500/2
		mesh	0	0.082	0
		mesh	0	-0.082	0
		mesh	0	0	0.20500/2
		mesh	0	0	0.20500
	endsegment

// Informations about Trunk segment
	segment	Trunk
		parent Pelvis
		RT
			1.00000	0.00000	0.00000	0
                        0.00000	1.00000	0.00000	0
			0.00000	0.00000	1.00000	0.20500
			0.00000	0.00000	0.00000	1.00000
		rotations	y
		mass	14.91158
		inertia
			0.16555	0.00000	0.00000
			0.00000	0.20687	0.00000
			0.00000	0.00000	0.13332
		com	0.00000	0.00000 0.16699
                mesh	0	0	0
                mesh	0	0.148	0.34000
                mesh	0	-0.148	0.34000
                mesh	0	0	0
	endsegment


	// Informations about Head segment
	segment	Head
	    parent Trunk
		RT
			1.00000	0.00000	0.00000	0.00
			0.00000	1.00000	0.00000	0.00
			0.00000	0.00000	1.00000	0.34000 
			0.00000	0.00000	0.00000	1.00000
		rotations	y
		mass	3.77189
		inertia
			0.01598	0.00000	0.00000
			0.00000	0.01598	0.00000
			0.00000	0.00000	0.01061
		com	0.00000	0.00000 0.10470
		mesh	0	0	0 
		mesh 	0	0	0.21400
	endsegment
	
// Informations about Bras segment
	segment	BrasD
		parent Trunk
		RT
                        1.00000	0.00000	0.00000	0.00000
                        0.00000	1.00000	0.00000	-0.148
			0.00000	0.00000	1.00000	0.34000
			0.00000	0.00000	0.00000	1.00000
		rotations	y
		mass	2.47288
		inertia
			0.07562	0.00000	0.00000
			0.00000	0.07570	0.00000
			0.00000	0.00000	0.00162
                com	0.00000	0.00000	-0.25478
		mesh	0	0	0
                mesh	0.0000	0.000	-0.65850
	endsegment

// Informations about Bras segment
	segment	BrasG
                parent Trunk
		RT
                        1.00000	0.00000	0.00000	0.0000
                        0.00000	1.00000	0.00000 0.148
                        0.00000	0.00000	1.00000	0.34000
                        0.00000	0.00000	0.00000	1.00000
                rotations y
		mass	2.47288
		inertia
			0.07562	0.00000	0.00000
			0.00000	0.07570	0.00000
			0.00000	0.00000	0.00162
                com	0.00000	0.00000	-0.25478
		mesh	0	0	0
                mesh	0.0000	0.000	-0.65850
	endsegment

// Informations about Cuisses segment
        segment	ThighL
                parent Pelvis
                RT
                        1.00000	0.00000	0.00000	0.000
                        0.00000	1.00000	0.00000	0.082
                        0.00000	0.00000	1.00000	0.0000
                        0.00000	0.00000	0.00000	1.00000
                rotations	y
                mass	6.47744
                inertia
                        0.07275	0.00000	0.00000
			0.00000	0.07275	0.00000
			0.00000	0.00000	0.01915
                com	0.00000	0.00000 -0.15110
                mesh	0	0	0
                mesh	0	0	-0.35350
        endsegment

// Informations about left leg segment
        segment	LegL
                parent ThighL
                rotations	y
                mass	3.20698
		RT
                        1.00000	0.00000	0.00000	0.000
                        0.00000	1.00000	0.00000	0.000
                        0.00000	0.00000	1.00000	-0.35350
                        0.00000	0.00000	0.00000	1.0000
                inertia
                        0.03877	0.00000	0.00000
			0.00000	0.03877	0.00000
			0.00000	0.00000	0.00408
                com	0.00000	0.00000	-0.16793
                mesh 	0	0	0
                mesh	0	0	-0.39050
        endsegment
        
// Informations about left foot segment
        segment	FootL
                parent LegL
                rotations	y
                mass	0.70991
		RT
                        1.00000	0.00000	0.00000	0.000
                        0.00000	1.00000	0.00000	0.000
                        0.00000	0.00000	1.00000	-0.39050
                        0.00000	0.00000	0.00000	1.0000
                inertia
                        0.00160	0.00000	0.00000
			0.00000	0.00178	0.00000
			0.00000	0.00000	0.00051
                com	0.00000	0.00000	-0.06997
                mesh	0	0	0
                mesh	0	0	-0.17450
                mesh 	-0.037	0	0.0250
                mesh	0	0	0
        endsegment

        
        
        marker ToeR
	  parent FootR
	  position 0 0 0
	endmarker
	
	marker TalR
	  parent FootR
	  position -0.037	0	0.0250+0.17450
	endmarker
	
	marker EpauleR
	  parent BrasD
	  position 0 0 0
	endmarker



        contact	PiedR
		parent	FootR
                position	0.0000	0.000	0.0000 
		axis	xz
	endcontact
        contact	PiedL
                parent	FootL
                position	0	0	-0.17450
		axis	xz
        endcontact
/*
        contact	TalR
		parent	FootR
                position	-0.037	0	0.0250+0.17450
		axis	xz
	endcontact
        contact	TalL
                parent	FootL
                position	-0.037	0	0.0250
		axis	xz
        endcontact
*/