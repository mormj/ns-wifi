function [varargout] = read_samples_from_file(filename,L,varargin)

filetype = 'complex_float';
nsamples_in = 0;
if length(varargin) > 0
    filetype = varargin{1};
end

if length(varargin) > 1
    nsamples_in = varargin{2};
end

nsamples_trash = 0;
if length(varargin) > 2
    nsamples_trash = varargin{3};
end



x = [];
for n = 1:L
   if (L > 1)
     filename_wr = [filename  '_' num2str(n-1) '.fc32']
   else
     filename_wr = [filename];
   end

	if (length(varargin) <= 3)
		fid = fopen(filename_wr,'r');
	else
		fid = varargin{4};
	end
   
    s = dir(filename_wr);

    if (strcmp(filetype,'complex_float'))
    %nsamples_trash
        nsamples = s.bytes/(2*4);  % gr_complex 4 byte float complex
        if nsamples_in ~= 0
          nsamples = nsamples_in;
        end

		if (nsamples_trash > 0)
            while(nsamples_trash > 0)
                if (nsamples_trash < 5e6)
                    A = fread(fid, [2,nsamples_trash],'single'); % throw away
                    nsamples_trash = 0;
                else
                    A = fread(fid, [2,5e6],'single'); % throw away
                    nsamples_trash = nsamples_trash - 5e6;
                end
            end
		end
        A = fread(fid, [2,nsamples], 'single');
		
		
		if (~isempty(A))
			xx = A(1,:) + A(2,:)*j;
			x = [x xx(:)];
		else
			x = [];
        end

	elseif (strcmp(filetype,'sc16') || strcmp(filetype,'ci16_le'))
    %nsamples_trash
        nsamples = s.bytes/(2*2);  % gr_complex 4 byte float complex
        if nsamples_in ~= 0
          nsamples = nsamples_in;
        end

		if (nsamples_trash > 0)
            while(nsamples_trash > 0)
                if (nsamples_trash < 5e6)
                    A = fread(fid, [2,nsamples_trash],'int16'); % throw away
                    nsamples_trash = 0;
                else
                    A = fread(fid, [2,5e6],'int16'); % throw away
                    nsamples_trash = nsamples_trash - 5e6;
                end
            end
		end
        A = fread(fid, [2,nsamples], 'int16');
		
		
		if (~isempty(A))
			xx = A(1,:) + A(2,:)*j;
			x = [x xx(:)];
		else
			x = [];
		end
    elseif (strcmp(filetype,'float'))
        nsamples = s.bytes/(4);  % float 4 byte float 
        if nsamples_in ~= 0
          nsamples = nsamples_in;
        end

		if (nsamples_trash > 0)
            while(nsamples_trash > 0)
                if (nsamples_trash < 5e6)
                    A = fread(fid, [1,nsamples_trash],'single'); % throw away
                    nsamples_trash = 0;
                else
                    A = fread(fid, [1,5e6],'single'); % throw away
                    nsamples_trash = nsamples_trash - 5e6;
                end
            end
		end
        
        A = fread(fid, [1,nsamples], 'single');

        xx = A(1,:);
        x = [x xx(:)];
    elseif (strcmp(filetype,'double'))
        nsamples = s.bytes/(8);  % float 4 byte float 
        if nsamples_in ~= 0
          nsamples = nsamples_in;
        end

        A = fread(fid, [1,nsamples], 'double');

        xx = A(1,:);
        x = [x xx(:)];

    elseif (strcmp(filetype,'u8'))
        nsamples = s.bytes;
        if nsamples_in ~= 0
          nsamples = nsamples_in;
        end
        
        A = fread(fid,[1,nsamples], 'uint8');
        xx = A(1,:);
        x = [x xx(:)];
    end
end

varargout{1} = x;

if nargout > 1
varargout{2} = fid;
else
fclose(fid);
end
