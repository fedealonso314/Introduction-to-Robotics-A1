<h2 align="left"> Final Project</h2>
<h1 align="center"> P3R Book Sorting Robot</h1>
<h2 align="center">Team A-1</h2>
<h2 align="left">Project details:</h2>
<details>
<summary> About the Robot

</summary>

</details>

<details>
<summary> Simulations

</summary>

https://github.com/user-attachments/assets/466b8830-91f7-4014-bd22-bce9394509c4


%%          BASIC SIMULATION 

clear 

clc



L1 = 0; 

L2 = 5; 

L3 = 5; 

L4 = 4; 



% theta d a alpha

DH = [0 0 0 0; 0 0 L1 0 ; 0 0 L2 0 ; 0 0 L3 0 ; 0 0 L4 0]; 





L(1) = Link('prismatic', 'theta', DH(1,1), 'a', DH(1,3), 'alpha', DH(1,4), 'modified');

L(2) = Link('revolute', 'd', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4), 'modified');

L(3) = Link('revolute', 'd', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4), 'modified');

L(4) = Link('revolute', 'd', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4), 'modified');

L(5) = Link('revolute', 'd', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4), 'modified');



bobot = SerialLink(L, 'name', 'bobot'); 





qi = [6 0 pi/6 0 0]; 

qd = [9 2*pi -pi/6 0 0]; 

q0 = [9 0 pi/6 0 0]; 



Ti = fkine(bobot, qi);

Td = fkine(bobot, qd);



% bobot.plot(qi, 'workspace', [-20, 20, -20, 20, -5, 20])



% Number of steps in animation

steps = 500;



% Generate trajectory from qi to qd

q_traj = jtraj(qi, q0, steps);

q_traj2 = jtraj(q0, qd, steps);



% Open a figure

figure;







% bobot.plot(qi, 'workspace', [-20 20 -20 20 -5 20])



% Animate



bobot.plot(q_traj, 'workspace', [-20 20 -20 20 -10 20], 'delay', 0.00000001);   % <-- makes animation as fast as possible

bobot.plot(q_traj2, 'workspace', [-20 20 -20 20 -10 20], 'delay', 0.00000001);

https://github.com/user-attachments/assets/0b08ce72-e509-4a36-8629-4c3a73730fc9
</details>

<details>
<summary> Mathematical Models
</summary>
  
#### DH-Parameters
  
#### Jacobian
</details>

</details>


<details>
<summary> CAD Models

</summary>

</details>

<details>
<summary>Team Members

</summary>

🤖 Federico Alonsexo

👩‍🔧 Giovana Bogado

👨‍🏭 Andres Fernandez

🛠️ Facundo Mendoza

</details>


