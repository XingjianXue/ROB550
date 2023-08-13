import numpy as np
u = 0
v = 0
d = 0
K = np.array([[904.5715942382812, 0.0,               635.9815063476562],
             [0.0,               905.2954711914062, 353.06036376953125], 
             [0.0,               0.0,               1.0               ]],dtype=np.float64)

pixel = np.array([[u],
                  [v], 
                  [1]],dtype=np.float64)

camera_coordinate = np.matmul(d*np.linalg.inv(K),pixel)

camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])

translation_matrix = np.array([[1,0,0,-30],
                               [0,1,0,300], 
                               [0,0,1,990],
                               [0,0,0,1]],dtype=np.float64)
rotation_matrix = np.array([[1,0                ,0                 ,0  ],
                            [0,np.cos(8*np.pi/9),-np.sin(8*np.pi/9),0  ], 
                            [0,np.sin(8*np.pi/9),np.cos(8*np.pi/9) ,0],
                            [0,0                ,0                 ,1  ]],dtype=np.float64)
H = np.matmul(translation_matrix,rotation_matrix)




world_coordinate = np.matmul(np.linalg.inv(H),camera_coordinate_homogenuous)


wx = int(world_coordinate[0,0])
wy = int(world_coordinate[1,0])
wz = int(world_coordinate[2,0])

print(wx,wy,wz)



