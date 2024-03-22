function pos = leg_pos(phi1,phi4)
%LEG_POS
%    POS = LEG_POS(PHI1,PHI4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-01-24 21:38:42

t2 = cos(phi1);
t3 = cos(phi4);
t4 = sin(phi1);
t5 = sin(phi4);
t6 = t2.*(3.0./2.0e+1);
t7 = t3.*(3.0./2.0e+1);
t8 = t4.*(3.0./2.0e+1);
t9 = t5.*(3.0./2.0e+1);
t12 = t2.*(5.4e+1./6.25e+2);
t13 = t3.*(5.4e+1./6.25e+2);
t14 = t4.*(5.4e+1./6.25e+2);
t15 = t5.*(5.4e+1./6.25e+2);
t10 = -t6;
t11 = -t9;
t16 = -t12;
t17 = -t14;
t18 = -t15;
t19 = t8+t11;
t21 = t7+t10+3.0./2.0e+1;
t23 = t14+t18;
t25 = t13+t16+5.4e+1./6.25e+2;
t20 = t19.^2;
t22 = t21.^2;
t24 = t23.^2;
t26 = t25.^2;
t27 = t20+t22;
t28 = t27.^2;
t30 = t25+t27;
t29 = -t28;
t31 = 1.0./t30;
t32 = t24+t26+t29;
t33 = sqrt(t32);
t34 = t15+t17+t33;
t35 = t31.*t34;
t36 = atan(t35);
t37 = t36.*2.0;
t38 = cos(t37);
t39 = sin(t37);
t40 = t38.*(3.6e+1./1.25e+2);
pos = [sqrt((t8+t39.*(3.6e+1./1.25e+2)).^2+(t6+t40-3.0./4.0e+1).^2);atan2(t4.*(3.0./2.0e+1)+t39.*(3.6e+1./1.25e+2),t6+t40-3.0./4.0e+1)];
end
