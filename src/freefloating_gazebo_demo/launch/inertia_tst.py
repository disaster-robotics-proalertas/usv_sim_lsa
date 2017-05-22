m = 0.05
w = 0.1
d = 0.02
h = 0.3

Ih = m*(pow(w,2)+pow(d,2))/12
Iw = m*(pow(d,2)+pow(h,2))/12
Id = m*(pow(h,2)+pow(w,2))/12

print(Iw, "  ", Id, "  ", Ih, "  ")
