function v = write_obj(obj, filename)
FV1 = obj.f.v;
FVN = obj.f.vn;
FVT = obj.f.vt;
V1 = obj.v;
VT = obj.vt;
N = obj.vn;

fileID = fopen(filename,'w');
for j=1:length(V1)
fprintf(fileID,'%s%12.5f%12.5f%12.5f\n','v', V1(j,1), V1(j,2), V1(j,3));
end
for j=1:length(VT)
fprintf(fileID,'%s %12.5f %12.5f\n','vt', VT(j,1), VT(j,2));
end
for j=1:length(N)
fprintf(fileID,'%s %12.5f %12.5f %12.5f\n','vn', N(j,1), N(j,2), N(j,3));
end
for j=1:length(FV1)
fprintf(fileID,'%s %i%c%i%c%i %i%c%i%c%i %i%c%i%c%i\n','f', FV1(j,1),'/',FVT(j,1),'/',FVN(j,1), FV1(j,2), '/', FVT(j,2), '/',FVN(j,2), FV1(j,3), '/', FVT(j,3), '/',FVN(j,3));
end
fclose(fileID);
end

