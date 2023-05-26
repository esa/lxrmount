using LinearAlgebra, DelimitedFiles
using StaticArrays
using DSP

Rot = SArray{Tuple{3,3},Float64,2,9}
Vec3 = SArray{Tuple{3},Float64,1,3}

#pv=readdlm("pv_wcube_2023-03-09T145829_720.txt", skipstart=1)
#pv=readdlm("pv_wcube_2023-03-12T011236_720.txt", skipstart=1)
pv=readdlm("pv_iss_2023-05-25T200000_720.txt", skipstart=1)

t=pv[:,1];p=pv[:,2:4];v=pv[:,5:7];

#clf();plot(sqrt.(sum(p.^2, dims=2))*1e-3);grid(true);
#clf();plot(p*1e-3);grid(true);

function Rx(th::Float64) ::Rot
    c = cos(th);  s = sin(th)
    return SA_F64[1 0 0
                  0 c -s
                  0 s c]
end

function Rz(th::Float64) ::Rot
    c = cos(th);  s = sin(th)
    return SA_F64[c -s 0
                  s c 0
                  0 0 1]
end

function topo2azel_p(p::Vec3)
    return [atan(p[2], p[1]), atan(p[3], hypot(p[1], p[2]))]
end


#Az_mnt=0.0; El_mnt=pi/2;
Az_mnt=float(0); El_mnt=-52.1868056*(pi/180) # mlyn Lange Voort, Oe, NL

Tmnt2wrld = Rx(pi/2-El_mnt)*Rz(Az_mnt)

N=length(t)
th=zeros(N,2)
Twrld2mnt = Tmnt2wrld'
for k=1:N
    th[k,:] = topo2azel_p(Twrld2mnt * p[k,:])
end


clf();plot(diff(unwrap(th,dims=1),dims=1)*180/pi)
