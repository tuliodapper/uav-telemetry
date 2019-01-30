function [ s, flag ] = setup_serial( comPort )
flag = 0;
s = serial(comPort);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',57600);
set(s,'Parity','none');
set(s, 'Timeout', 2);
fopen(s);
flag = 1;
end