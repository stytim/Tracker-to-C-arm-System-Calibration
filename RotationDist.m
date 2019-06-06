function [deg, euldeg] = RotationDist(A,B)
    num = size(A,3);
    for i=1:num
        R = A(:,:,i) * B(:,:,i)';
        angle(:,:,i) = acos((trace(R) - 1)/2);
        euler_rot(i,1:3) = rotm2eul(R);
    end
    deg = mean(abs(angle),3);
    for i = 1:3 
        mean_euler_rotation(i) = sqrt(sum(euler_rot(:,i)' * euler_rot(:,i)) / sum(num));
    end 
    euldeg = rad2deg(mean_euler_rotation);
return