function q = Matrix_to_quaternion( R )
    %Transforms from a 3x3 rotation matrix to a quaternion.
    w = sqrt((1+R(1,1)+R(2,2)+R(3,3)))/2; %quaternion calculations.
    x = (R(2,3) - R(3,2))/(4*w);
    y = (R(3,1) - R(1,3))/(4*w);
    z = (R(1,2)-R(2,1))/(4*w);
    q =[w,x,y,z];
end

