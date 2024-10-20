clear; 
clc;

msg_t_x = [1.09083, 1.09433, 1.07652, 1.08535, 1.08187, 1.07508, 1.07989, 1.0784, 1.07559, 1.07443, 1.02678, 1.07401, 0.730778, 1.03631, 1.02198, 1.0304, 1.04342, 1.02191, 1.03889, 1.00833, 1.00557, 0.997332, 1.03541, 0.999945, 1.00865, 1.00874, 0.992028, 0.993198, 1.00176, 0.990253, 0.978302, 0.994899, 0.98711, 0.973212, 0.968232, 0.957846, 0.939113, 0.95342, 0.94504, 0.955682, 0.912044, 0.897565, 0.935837, 0.94458, 0.9245, 0.908048, 0.890588, 0.902533, 0.894277, 0.907784, 0.902142, 0.863516, 0.816268, 0.783751, 0.74321, 0.698669, 0.652953, 0.616612, 0.57498, 0.539379, 0.49473, 0.454851, 0.41021, 0.380092, 0.346428, 0.0507246, 0.040947, 0.0310352, 0.0210444, 0.010454, 3.61883e-17, -0.0104016, -0.0208699, -0.138444, -0.153679, -0.162378, -0.174022, -0.171863, -0.173216, -0.184538, -0.195703, -0.208169, -0.199007, -0.214224, -0.232232, -0.246199, -0.255482, -0.265306, -0.274141, -0.873674, -0.946116, -0.969928, -1.01907, -1.044, -1.11094, -1.12184, -1.17642, -1.22407, -1.26416, -1.24963, -1.36677, -1.55486, -1.60458, -1.6709, -1.72275, -1.77272, -1.81993, -1.873, -1.93515, -1.963, -1.97103, -2.17255, -2.1683, -2.27253, -2.38046, -2.43074, -2.47911, -2.52294, -2.80037, -2.86741, -2.85126, -2.8325, -2.78796, -3.2051, -3.36331, -3.45229, -3.44821, -3.41296, -3.42997, -3.40639, -3.37172, -3.38861, -3.36906, -3.30482, -3.33051, -3.31744, -3.33917, -3.29147, -3.1628, -3.21146, -3.84166, -3.76843, -3.872, -3.88441, -3.76471, -3.89466, -3.7658, -3.90608, -3.85377, -3.82031, -3.84719, -3.69401, -3.79225, -3.46439, -3.6913, -3.67044, -3.55853, -3.56512, -3.61216, -3.68233, -3.064, -2.77622, -2.74368, -2.68131, -2.69199, -1.86328, -1.68001, -1.62781, -1.54677, -1.49755, -1.45086, -1.38, -1.33274, -1.2394, -1.20035, -1.15467, -1.08782, -1.04206, -0.995583, -0.932396, -0.89377, -0.841712, -0.801549, -0.749057, -0.696722, -0.656568, -0.612884, -0.569242, -0.525711, -0.466762, -0.426649, -0.383762, -0.344625, -0.305207, -0.267381, -0.226513, -0.186775, -0.150325, -0.111057, -0.0747547, -0.0369293, -3.87233e-16, 0.0366326, 0.0730446, 0.108859, 0.145791, 0.181633, 0.215329, 0.252391, 0.286975, 0.321004, 0.355805, 0.389823, 0.427674, 0.46115, 0.496666, 0.529803, 0.566435, 1.74877, 1.84809, 1.56953, 1.56712, 1.55158, 1.5564, 1.56928, 1.53381, 1.42977, 1.42717, 1.41078, 1.37853, 1.38112, 1.33368, 1.3398, 1.28997, 1.33292, 1.32598, 1.30737, 1.31817, 1.25214, 1.27631, 1.26775, 1.26353, 1.25858, 1.26438, 1.23761, 1.22506, 1.21355, 1.22598, 1.2031, 0.92533, 1.21646, 1.21502, 1.21355, 1.18199, 1.17658, 1.15077, 1.12735, 1.14248, 1.13476, 1.14632, 1.15658, 1.13485, 1.10768, 1.12083, 1.10179, 1.08635, 1.11747, 1.08934, 1.16782];
msg_t_y = [0.0190406, 0.0382149, 0.0564182, 0.075895, 0.0946511, 0.112995, 0.132594, 0.15156, 0.170357, 0.18945, 0.199586, 0.228287, 0.168713, 0.505442, 0.520727, 0.547873, 0.578378, 0.59, 0.624226, 0.630074, 0.653022, 0.672709, 0.725001, 0.726503, 0.788047, 0.816858, 0.83241, 0.863374, 0.901988, 0.923426, 0.944735, 0.994899, 1.02218, 1.04364, 1.07533, 1.10188, 1.11919, 1.17738, 1.2096, 1.26823, 1.35216, 1.38213, 1.49765, 1.57204, 1.60128, 1.63816, 1.67495, 1.77132, 1.83354, 1.94675, 2.02624, 2.03432, 2.02033, 2.04174, 2.04195, 2.02908, 2.00958, 2.01685, 2.00519, 2.01299, 1.98425, 1.97018, 1.92989, 1.9554, 1.96469, 0.579785, 0.58557, 0.592187, 0.602633, 0.598909, 0.591, 0.595909, 0.597636, 0.874104, 0.871555, 0.835365, 0.81871, 0.744419, 0.694732, 0.688705, 0.682496, 0.680889, 0.61248, 0.622151, 0.638051, 0.64137, 0.632339, 0.625023, 0.61573, 1.7913, 1.85686, 1.82417, 1.83845, 1.80826, 1.84891, 1.79532, 1.81153, 1.81476, 1.80541, 1.71997, 1.74938, 1.78866, 1.78206, 1.79182, 1.78396, 1.77272, 1.75749, 1.7466, 1.74242, 1.70641, 1.65389, 1.69738, 1.63393, 1.65109, 1.66681, 1.63955, 1.60995, 1.57651, 1.68263, 1.6555, 1.58048, 1.50606, 1.42054, 1.56323, 1.56834, 1.53706, 1.46368, 1.37893, 1.31664, 1.23982, 1.16098, 1.10103, 1.03003, 0.947641, 0.892408, 0.827131, 0.770907, 0.699623, 0.614787, 0.566267, 0.134154, 0.0657781, 4.74183e-16, -0.0678026, -0.131466, -0.20411, -0.263331, -0.341738, -0.405048, -0.469075, -0.540688, -0.651354, -0.87551, -0.993397, -1.27102, -1.33593, -1.43774, -1.5133, -1.60824, -1.7171, -1.769, -2.58887, -2.94224, -3.0845, -3.20819, -2.56458, -2.3993, -2.41333, -2.38182, -2.39658, -2.41464, -2.39023, -2.40433, -2.33098, -2.35582, -2.36742, -2.33284, -2.3405, -2.34545, -2.30776, -2.32835, -2.31258, -2.32787, -2.30536, -2.27887, -2.28973, -2.28731, -2.28311, -2.2771, -2.19594, -2.19492, -2.17643, -2.17588, -2.17166, -2.17765, -2.15513, -2.13485, -2.14975, -2.11909, -2.1407, -2.11568, -2.108, -2.09868, -2.09172, -2.07715, -2.08491, -2.07607, -2.04872, -2.05556, -2.04193, -2.02674, -2.01787, -2.00546, -2.01205, -1.99746, -1.99202, -1.97725, -1.97539, -3.43216, -3.33405, -1.9382, -1.86762, -1.78489, -1.72855, -1.68284, -1.5883, -1.42977, -1.3782, -1.31557, -1.24124, -1.20059, -1.11909, -1.08495, -1.00784, -1.00443, -0.96338, -0.915428, -0.889117, -0.813146, -0.797529, -0.761741, -0.7295, -0.697641, -0.672283, -0.630593, -0.5975, -0.565886, -0.545841, -0.510686, -0.373857, -0.466953, -0.442232, -0.394306, -0.361371, -0.33738, -0.286919, -0.260268, -0.242841, -0.220575, -0.202126, -0.183185, -0.159492, -0.136006, -0.117804, -0.0963942, -0.0759648, -0.0585639, -0.0380405, -0.0203844];

msg_t_minus_1_x = [1.17082, 1.13231, 1.13744, 1.12525, 1.1257, 1.10292, 1.11761, 1.08335, 1.08843, 1.06556, 1.0739, 1.0877, 1.0455, 1.05374, 1.05769, 1.04297, 1.05002, 1.0338, 1.03062, 0.985738, 0.995197, 0.992087, 0.995066, 0.974753, 0.955248, 0.937339, 0.872352, 0.664241, 0.824967, 0.815731, 0.529447, 0.471303, 0.4435, 0.383311, 0.42046, 0.403112, 0.387547, 0.37036, 0.336704, 0.309271, 0.28948, 0.273353, 0.217106, 0.139711, 0.110006, 0.0804404, 0.0537452, -0.095425, -0.10471, -0.113913, -0.123084, -0.132046, -0.140799, -0.150891, -0.162902, -0.1763, -0.185899, -0.197004, -0.20857, -0.220643, -0.234048, -0.237941, -0.264559, -0.332855, -0.341306, -0.3495, -0.359497, -0.366174, -0.370899, -0.378014, -0.395768, -0.399106, -0.409234, -0.422959, -0.43486, -0.448666, -0.455305, -0.466384, -0.479445, -0.486261, -0.497096, -0.471168, -1.63934, -1.66232, -1.72496, -1.73783, -1.7944, -1.78903, -1.81891, -1.88365, -2.01157, -2.04863, -2.09318, -2.12795, -2.16322, -2.1999, -2.2326, -2.30837, -2.37156, -2.31691, -2.35876, -2.32181, -2.37366, -2.38933, -2.43756, -2.78476, -2.78766, -2.83692, -2.85752, -3.18876, -3.26097, -3.15631, -3.11912, -3.11043, -3.67242, -3.64393, -3.70086, -3.71493, -3.68295, -3.63478, -3.57446, -3.544, -3.50547, -3.49787, -3.42031, -3.41366, -3.38108, -3.36347, -3.29724, -3.22827, -3.6919, -3.79615, -3.74922, -3.742, -3.73066, -3.64639, -3.62067, -3.59392, -3.5962, -3.51213, -3.64291, -3.48779, -3.39017, -3.48294, -3.36239, -3.40008, -3.44085, -3.32968, -3.29609, -3.19723, -3.23361, -3.17437, -3.13708, -3.20288, -3.06617, -3.06436, -3.18041, -3.05341, -2.95846, -2.96525, -2.93785, -2.91964, -2.6786, -2.03491, -1.97425, -1.72006, -1.68267, -1.5666, -0.915926, -0.868636, -0.786945, -0.735822, -0.673752, -0.61839, -0.562401, -0.508124, -0.456, -0.401724, -0.359902, -0.311742, -0.266757, -0.216756, -0.169438, -0.125502, -0.0828514, -0.0411179, -4.25075e-16, 0.0402278, 0.081595, 0.120948, 0.160649, 0.195316, 0.231217, 0.269331, 0.340558, 0.411766, 0.44202, 0.477571, 0.545849, 0.57746, 0.614858, 0.647082, 0.675554, 0.706614, 0.741463, 0.772439, 0.799827, 0.840318, 0.862141, 0.902168, 0.925233, 0.965703, 0.998223, 1.0235, 1.0687, 2.60894, 2.5875, 2.03492, 1.96429, 1.93893, 1.87777, 1.8694, 1.84111, 1.75976, 1.71733, 1.68714, 1.6467, 1.63321, 1.53694, 1.55069, 1.49091, 1.48569, 1.49833, 1.4984, 1.43961, 1.4041, 1.44465, 1.41916, 1.37076, 1.35133, 1.36618, 1.3534, 1.31561, 1.3625, 1.02912, 1.28399, 1.27249, 1.23619, 1.23229, 1.16316, 1.14643, 1.21126, 1.17982];
msg_t_minus_1_y = [0.0204368, 0.0395411, 0.0596107, 0.0786853, 0.098486, 0.115922, 0.137225, 0.152255, 0.172391, 0.187887, 0.208745, 0.231198, 0.241372, 0.262727, 0.283407, 0.299067, 0.321024, 0.335901, 0.354869, 0.358779, 0.38202, 0.400829, 0.42238, 0.433988, 0.44544, 0.477598, 0.463838, 0.3835, 0.769294, 0.787743, 1.45464, 1.54156, 1.54667, 1.43054, 1.68637, 1.74607, 1.82327, 1.90534, 1.90954, 1.95266, 2.05976, 2.22628, 2.06562, 1.5969, 1.57316, 1.53489, 1.53906, 0.60249, 0.593839, 0.586031, 0.579063, 0.571955, 0.564712, 0.563135, 0.568106, 0.576652, 0.539891, 0.541263, 0.543344, 0.546111, 0.551382, 0.534424, 0.567349, 0.62601, 0.615732, 0.605352, 0.598303, 0.586001, 0.571135, 0.560429, 0.565215, 0.549323, 0.543072, 0.541363, 0.537008, 0.534699, 0.523768, 0.517972, 0.514142, 0.503538, 0.497096, 0.455001, 1.37557, 1.34612, 1.34768, 1.30955, 1.30371, 1.25269, 1.22687, 1.22326, 1.25697, 1.23094, 1.2085, 1.17954, 1.15021, 1.1209, 1.08891, 1.07641, 1.05589, 0.98347, 0.952999, 0.891261, 0.863943, 0.822711, 0.792011, 0.851386, 0.799348, 0.760152, 0.71246, 0.677792, 0.633868, 0.556542, 0.49402, 0.437143, 0.450917, 0.382992, 0.323784, 0.259773, 0.193015, 0.126929, 0.0623924, 4.34015e-16, -0.0611881, -0.122148, -0.179251, -0.238707, -0.295807, -0.353515, -0.40485, -0.453704, -0.717633, -0.876409, -0.934786, -1.00267, -1.06975, -1.11481, -1.17643, -1.23748, -1.30891, -1.34818, -1.47183, -1.48048, -1.5094, -1.62412, -1.63995, -1.73243, -1.82953, -1.84567, -1.903, -1.92109, -2.02058, -2.06146, -2.11599, -2.24268, -2.22771, -2.30916, -2.48481, -2.4726, -2.48245, -2.57766, -2.73959, -2.91964, -2.77377, -2.80082, -2.81952, -3.68867, -3.77934, -3.87748, -2.81893, -2.84118, -2.7444, -2.74613, -2.70227, -2.67854, -2.64589, -2.61407, -2.58611, -2.53638, -2.56083, -2.53893, -2.53802, -2.47754, -2.42308, -2.39471, -2.37255, -2.35564, -2.314, -2.30465, -2.33658, -2.30783, -2.29739, -2.23247, -2.19988, -2.19353, -2.1502, -2.11835, -2.07954, -2.06859, -2.03714, -2.01384, -2.01111, -1.99151, -1.96195, -1.9414, -1.93158, -1.91185, -1.88427, -1.88738, -1.84887, -1.84972, -1.81587, -1.81622, -1.80084, -1.77275, -1.77862, -2.89752, -2.77476, -1.53342, -1.42714, -1.35766, -1.26657, -1.214, -1.15045, -1.05737, -0.9915, -0.935198, -0.875564, -0.832165, -0.749615, -0.7231, -0.663794, -0.63064, -0.605364, -0.575181, -0.523975, -0.483469, -0.469397, -0.43388, -0.393059, -0.362088, -0.340626, -0.312457, -0.279641, -0.264843, -0.181462, -0.203365, -0.178837, -0.129929, -0.107812, -0.0813361, -0.0600817, -0.0422982, -0.0205938];

% have to remove all values == 0, as that means there is no laser data for
% that point, need to note this in c++ implementation too
msg_t_x(msg_t_x==0) = [];
msg_t_y(msg_t_y==0) = [];
msg_t_minus_1_x(msg_t_minus_1_x==0) =[];
msg_t_minus_1_y(msg_t_minus_1_y==0) =[];

plot(msg_t_x,msg_t_y,'.r')
hold on
plot(msg_t_minus_1_x,msg_t_minus_1_y,'.b')


size_t_minus_1 = length(msg_t_minus_1_x)
size_t = length(msg_t_x)

%calculate centroid for each msg:
centroid_t_x = 0;
centroid_t_y = 0;
for i = 1:size_t
  if (msg_t_x(i) > 0.01 || msg_t_x(i) < -.01 && msg_t_y(i) > 0.01 || msg_t_y(i) < -.01)
    centroid_t_x = centroid_t_x + msg_t_x(i);
    centroid_t_y = centroid_t_y + msg_t_y(i);
  end;
end;

centroid_t_x  = centroid_t_x / size_t;
centroid_t_y  = centroid_t_y / size_t;

centroid_t_minus_1_x = 0;
centroid_t_minus_1_y = 0;
for j = 1:size_t_minus_1
  if (msg_t_minus_1_x(j) > 0.01 || msg_t_minus_1_x(j) < -.01 && msg_t_minus_1_y(j) > 0.01 || msg_t_minus_1_y(j) < -.01)
    centroid_t_minus_1_x = centroid_t_minus_1_x + msg_t_minus_1_x(j);
    centroid_t_minus_1_y = centroid_t_minus_1_y + msg_t_minus_1_y(j);
  end;
end;

centroid_t_minus_1_x = centroid_t_minus_1_x / size_t_minus_1;
centroid_t_minus_1_y = centroid_t_minus_1_y / size_t_minus_1;
%plot(centroid_t_x, centroid_t_y, 'xr')
%plot(centroid_t_minus_1_x, centroid_t_minus_1_y, 'xb')

%Variance Vector generation

for i = 1:size_t
  if (msg_t_x(i) > 0.001 || msg_t_x(i) < -.01 && msg_t_y(i) > 0.001 || msg_t_y(i) < -.01)
    prime_t_x(i) = msg_t_x(i) - centroid_t_x;
    prime_t_y(i) = msg_t_y(i) - centroid_t_y;
  end;
end;

for j = 1:size_t_minus_1
  if (msg_t_minus_1_x(j) > 0.01 || msg_t_minus_1_x(j) < -.001 && msg_t_minus_1_y(j) > 0.01 || msg_t_minus_1_y(j) < -.01)
    prime_t_minus_1_x(j) = msg_t_minus_1_x(j) - centroid_t_minus_1_x;
    prime_t_minus_1_y(j) = msg_t_minus_1_y(j) - centroid_t_minus_1_y;
  end;
end;

centroid_t_x
centroid_t_y
centroid_t_minus_1_x
centroid_t_minus_1_y

% make kd tree
t_points = [msg_t_x(:) , msg_t_y(:)];
t_minus_1_points = [msg_t_minus_1_x(:) , msg_t_minus_1_y(:)];

size_t_p = size(t_points);

[Idx, Dist] = knnsearch(t_points, t_minus_1_points);
for i = 1:size_t_minus_1
    index = Idx(i);
    %plot( [msg_t_minus_1_x(i), msg_t_x(index)], [msg_t_minus_1_y(i), msg_t_y(index)] );
end


if (size_t > size_t_minus_1)
    multiplication_iterations = size_t_minus_1
else
    multiplication_iterations = size_t
end

prime_t = [prime_t_x(:), prime_t_y(:)];
prime_t_minus_1 = [prime_t_minus_1_x(:), prime_t_minus_1_y(:)];
w = zeros(2,2);

for i= 1:multiplication_iterations
    w_loop = prime_t(i,:) .* prime_t_minus_1(i,:).';
    w = w + w_loop;
end
w;

[U, S, V] = svd(w);
centroid_t_x;
centroid_t = [centroid_t_x; centroid_t_y];
centroid_t_minus_1 = [centroid_t_minus_1_x; centroid_t_minus_1_y];
R = U * V;
T = centroid_t - R * centroid_t_minus_1;
Trans = zeros(3,3);
Trans(1,1) = R(1,1);
Trans(1,2) = R(1,2);
Trans(2,1) = R(2,1);
Trans(2,2) = R(2,2);
Trans(1,3) = T(1,1);
Trans(2,3) = T(2,1);
Trans(3,1) = 0;
Trans(3,2) = 0;
Trans(3,3) = 1;
Trans
size(Trans);


msg_t_minus_1 = [msg_t_minus_1_x(:), msg_t_minus_1_y(:)];
col_of_1s = ones(size_t_minus_1, 1);
msg_t_minus_1 = [msg_t_minus_1 col_of_1s];
size(msg_t_minus_1(2,:).');

for j = 1:size_t_minus_1
    %size(msg_t_minus_1(j,:).')
    trans_msg(j,:) = Trans * msg_t_minus_1(j,:).';
    
end

%plot(trans_msg(:,1), trans_msg(:,2), '.k')

actual_trans = [0.992595,  0.121467,  0.266009;
                -0.121467,  0.992595,  0.305868;
                    0,         0,         1];
for i = 1:size_t_minus_1
    trans_msg_actual(j,:) = actual_trans * msg_t_minus_1(j,:).';
end
plot(trans_msg_actual(:,1), trans_msg_actual(:,2), '.g')
hold off
 
    