%-----------------------------------------%
%--- The class of the Koopman operator ---%
%-----------------------------------------%
classdef koopman
    properties
        params struct;  % system identification parameters
        lift struct;    % lifting functions
        basis struct;   % observables for the system
      
        model;          % Koopman-based model matrices
        KPmodel;        % Koopman-based model
        
        type;           % the type of Koopman model
        degree;         % the maximum dimension of the lifting function
        delay;          % the number of past states used in the lifting function construction
        traindata;      % split data for training the model
        valdata;        % split data for validating the model
        snapshotPairs;  % shuffle training data pairs
        horizon;        % the length of the prediction horizon 
        name;           % the type of the robot to be optimized
        optimization;   % determine whether the current stage is during controller optimization
        matrixD;        % the matrix D delivered by BOHB for Koopman operator optimization
        datanoise;      % choose whether to identify the Koopman-based model with data sampling noise
    end
    
    methods
        % class initialization
        function obj = koopman(data, varargin) 
            % extract model data
            data = data.data_collection;
            datatrain = data.train;
            dataval = data.val;
            
            % basic parameters
            obj.params = struct;  
            obj.params.n = size(data.train{1, 1}.y, 2);  % dimension of system state
            obj.params.m = size(data.train{1, 1}.u, 2);  % dimension of input
            obj = obj.parse_args(varargin{:});
            obj.params.nd = obj.delay;
            obj.params.nzeta = obj.params.n * (obj.delay + 1) + obj.params.m * obj.delay;
            obj.horizon = obj.horizon + 1;
            
            % initialize structs
            obj.lift = struct;  
            obj.basis = struct; 
     
            % define the lifting functions
            obj = obj.def_observables(obj.degree, obj.matrixD);
            
            % merge the training data
            datatrain_merged = obj.merge_trials(datatrain);
            rng(0)
            if strcmp(obj.datanoise, 'Yes')     
                for i = 1 : size(datatrain_merged.x, 2)
                    nor_factor = max(datatrain_merged.x(:, i)) - min(datatrain_merged.x(:, i));
                    datatrain_merged.x(:, i) = datatrain_merged.x(:, i) + 0.02 * nor_factor * randn(size(datatrain_merged.x, 1), 1);
                end
                datatrain_merged.y = datatrain_merged.x;
            end
            obj.traindata = datatrain_merged;
           
            % extract the validation data
            obj.valdata = dataval;
            
            % shuffle the training data
            obj.snapshotPairs = obj.get_snapshotPairs(obj.traindata);
        end
        
        % attribute assignment
        function obj = parse_args(obj, varargin)
            for idx = 1 : 2 : length(varargin)
                obj.(varargin{idx}) = varargin{idx + 1} ;
            end
        end
        
        %% Data Processing
        % merge data
        function data_merged = merge_trials(~, data)
            if iscell(data)
                data_merged = data{1};
                for i = 2 : length(data)
                    data_fields = fields(data{i});
                    for j = 1 : length(data_fields)
                        if isa(data{i}.(data_fields{j}), 'numeric')
                            data_merged.(data_fields{j}) = [data_merged.(data_fields{j}); data{i}.(data_fields{j})];
                        end
                    end
                end
            else
                data_merged = data;
            end
        end
        
        %% Save Linear Model
        function obj = save_model(obj, ~)     
            if strcmp(obj.optimization, 'Yes') == 1
                classname = ['model_', obj.name];
                obj.params.classname = classname;  
                koopman_model = obj;
                dirname = ['model_koopman'];
            else
                classname = [obj.name, '_type-',  obj.type, '_degree-', num2str(obj.degree), '_delay-', num2str(obj.delay)];
                obj.params.classname = classname;  
                koopman_model = obj;
                dirname = ['model_environment'];
            end
            fname = [dirname, filesep, classname, '.mat'];
            save(fname, 'koopman_model');
        end
        
        %% Save Linear Model to be Optimized
        function obj = save_model_tem(obj, ~)     
            if strcmp(obj.optimization, 'Yes') == 1
                classname = ['model_', obj.name];
                obj.params.classname = classname;  
                koopman_model = obj;
                dirname = ['model_koopman_tem'];
            else
                classname = [obj.name, '_type-',  obj.type, '_degree-', num2str(obj.degree), '_delay-', num2str(obj.delay)];
                obj.params.classname = classname;  
                koopman_model = obj;
                dirname = ['model_environment'];
            end
            fname = [dirname, filesep, classname, '.mat'];
            save(fname, 'koopman_model');
        end
        
        %% Determine the Lifting Function Dictionary
        % determine lifting function properties
        function obj = def_observables(obj, degree, D)
            % define the state variable with delays
            x = sym('x', [obj.params.n, 1], 'real');                   % state variable
            xd = sym('xd', [obj.params.nd * obj.params.n, 1], 'real'); % state delays
            zeta = [x; xd];                                           % state variable with delays
            u = sym('u', [obj.params.m, 1], 'real');                   % input vector
            obj.params.zeta = zeta; 
            obj.params.x = x;
            obj.params.u = u;
            
            obj = obj.def_polyLift(degree);
            fullBasis = [obj.basis.poly(1 : end)];
            
            % optimize the lifting functions
            if strcmp(obj.optimization, 'Yes') == 1
                fullBasis = D * fullBasis;
            end
            
            % add a constant term to the end of the lifting function dictionary
            fullBasis = [fullBasis; 1];
            
            % save lifting function properties
            obj.params.zeta = zeta;
            obj.basis.full = fullBasis;
            obj.lift.full = matlabFunction(fullBasis, 'Vars', {[zeta]});
            obj.params.N = length(fullBasis) + length(u); 
            obj.params.Nx = length(fullBasis) ; 
        end
        
        % define polynomial lifting functions
        function [obj, polyBasis] = def_polyLift(obj, degree)
            % calculate the dimention of polynomial lifting functions
            zeta = obj.params.zeta;
            nzeta = length(zeta);
            N = prod((nzeta + 1) : (nzeta + degree)) / factorial(degree); 

            % determine polynomial lifting functions
            exponents = [];
            for i = 1 : degree
                exponents = [exponents; partitions(i, ones(1, nzeta))];
            end
            for i = 1 : N - 1
                polyBasis(i, 1) = obj.get_monomial(zeta, exponents(i, :));
            end
            
            % save polynomial lifting functions
            obj.lift.poly = matlabFunction(polyBasis, 'Vars', {zeta});
            obj.basis.poly = polyBasis;   
        end
        
        % create vector of order monomials (column vector)
        function [monomial] = get_monomial(~, x, exponents)
            n = length(x);
            monomial = x(1)^exponents(1);
            for i = 2 : n
                monomial = monomial * x(i)^exponents(i);
            end
        end

        %% Fit Koopman Operator and A, B, C System Matrices
        % organize data format
        function [data_out, zeta] = get_zeta(obj, data_in)
            data_out = data_in;
            for i = obj.params.nd + 1 : size(data_in.y, 1)
                ind = i - obj.params.nd;    % current timestep index
                y = data_in.y(i, :);
                ydel = zeros(1, obj.params.nd * obj.params.n);
                for j = 1 : obj.params.nd
                    fillrange_y = obj.params.n * (j - 1) + 1 : obj.params.n * j;
                    ydel(1, fillrange_y) = data_in.y(i - j, :);
                end
                data_out.zeta(ind, :) = [y, ydel];
                data_out.uzeta(ind, :) = data_in.u(i, :);
            end
            zeta = data_out.zeta;
        end
        
        % acquire random snapshots
        function snapshotPairs = get_snapshotPairs(obj, data)
            % check if data has a zeta field, create one if not
            if ~ismember('zeta', fields(data))
                [data, ~] = obj.get_zeta(data);
            end
            
            % separate data into 'before' and 'after' time step
            before.t = data.t(obj.params.nd + 1 : end-1);
            before.zeta = data.zeta(1:end-1, :);
            after.t = data.t(obj.params.nd + 2 : end);
            after.zeta = data.zeta(2:end, :);
            u = data.uzeta(1:end-1, :); 
            
            % remove invalid data from combining different datasets
            i = 1;
            goodpts = [];
            while i <= length(before.t)
                if before.t(i,1) > after.t(i,1)
                    i = i + obj.params.nd;
                else
                    goodpts = [goodpts; i];
                end
                i = i + 1;
            end
            before.zeta = before.zeta(goodpts, :);
            after.zeta = after.zeta(goodpts, :);
            u = u(goodpts, :);
                
            % random sampling without repetition
            total = size(before.zeta, 1);
            s = RandStream('mlfg6331_64'); 
            index = datasample(s, 1 : total, size(before.zeta, 1), 'Replace', false);
            
            % save snapshotPairs
            snapshotPairs.alpha = before.zeta(index, :); 
            snapshotPairs.beta = after.zeta(index, :);
            snapshotPairs.u = u(index, :);
        end
        
        % calculate the Koopman operator
        function [KPmodel] = get_Koopman(obj,  snapshotPairs, varargin)
            % extract snapshot
            [x, y, u] = deal(snapshotPairs.alpha, snapshotPairs.beta, snapshotPairs.u);

            % organize the data corresponding to the equation (9)
            N = obj.params.N ;
            Px = zeros(N, N);
            Py = zeros(N, N);  
            for i = 1:length(x)
                psix = obj.lift.full([x(i, :)'])';   
                psiy = obj.lift.full([y(i, :)'])';
                Px = Px + [psix, u(i, :)]' * [psix, u(i, :)];
                Py = Py + [psiy, u(i, :)]' * [psix, u(i, :)];
            end
            
            % solve the minimum-norm closed-form solution, corresponding to the equation (8)
            UT = Py * pinv(Px);
            
            % save the Koopman operator
            KPmodel.K = UT; 
            KPmodel.Px = Px(:, 1 : obj.params.N);   
            KPmodel.Py = Py(:, 1 : obj.params.N);
            KPmodel.u = u; 
            KPmodel.alpha = snapshotPairs.alpha;
            KPmodel.beta = snapshotPairs.beta;
        end
        
        % calculate the model matrices A, B, C
        function [out, obj] = get_model(obj, KPmodel)
            % extract the model matrices A, B, C
            UT = KPmodel.K;    
            A = UT(1 : obj.params.Nx, 1 : obj.params.Nx);
            B = UT(1 : obj.params.Nx, obj.params.Nx + 1 : end);
            Cy = [eye(obj.params.n), zeros(obj.params.n, obj.params.Nx - obj.params.n)];
            
            % save the model matrices A, B, C
            out.A = A; 
            out.B = B;  
            out.C = Cy;
            out.sys = ss(out.A, out.B, Cy, 0);  
            out.params = obj.params;     
            out.K = KPmodel.K; 
            obj.model = out;
        end
        
        % train the Koopman model
        function obj = train_model(obj)
            obj.KPmodel = obj.get_Koopman(obj.snapshotPairs);
            obj.model = obj.get_model(obj.KPmodel);
        end
        
        %% Validate Koopman Model
        % calculate prediction results
        function results = val_model(obj, model, valdata)
            % extract validation data
            index = obj.params.nd + 1;  
            treal = valdata.t(index : end);   
            yreal = valdata.y(index : end, :);
            ureal = valdata.u(index : end, :);
            [~, zetareal] = obj.get_zeta(valdata);
            zreal = zeros(size(zetareal, 1), obj.params.Nx);
            for i = 1 : size(zetareal, 1)
                zreal(i, :) = obj.lift.full([zetareal(i, :)'])';
            end
            
            % predict the future state based on the current state
            prey = zeros(size(yreal, 1) - obj.horizon + 1, obj.params.n * obj.horizon);
            prey(:, 1 : obj.params.n) = yreal(1 : size(yreal, 1) - obj.horizon + 1, :); 
            pret = zeros(size(yreal, 1) - obj.horizon + 1, obj.horizon);
            for i = 1 : length(treal) - obj.horizon + 1
                [valdata_zeta, ~] = get_zeta(obj, valdata);
                zpre = obj.lift.full([valdata_zeta.zeta(i, :)'])';
                for j = 2 : obj.horizon 
                    zpre = (model.A * zpre' + model.B * ureal(i + j - 2, :)')';
                    if strcmp(obj.name, 'AUV')
                        if zpre(1, 3) > pi
                            zpre(1, 3) = zpre(1, 3) - 2 * pi;
                        elseif zpre(1, 3) < - pi
                            zpre(1, 3) = zpre(1, 3) + 2 * pi;
                        end
                    end
                    prey(i, obj.params.n * (j - 1) + 1 : obj.params.n * j) = (model.C * zpre')';
                    pret(i, :) = treal(i : i + obj.horizon - 1);
                end
            end
            
            % save prediction results
            results = struct;
            results.real.t = treal;
            results.real.u = ureal;
            results.real.y = yreal;
            results.real.z = zreal;
            results.sim.prey = prey;
            results.sim.pret = pret;
        end
        
        % calculate prediction errors based on the attenuation factor
        function [datalength, preydata, realdata] = get_error(obj, simdata, realdata)
            % extract real data
            real = zeros(size(simdata.prey, 1), size(simdata.prey, 2));
            for i = 1 : size(simdata.pret, 1)
                for j = 1 : size(simdata.pret, 2)
                    real(i, obj.params.n * (j - 1) + 1 : obj.params.n * j) = realdata.y(i + j - 1, :);
                end
            end
            
            % adjust the weight of different prediction steps through the attenuation factor 
            beta = 0.9; % attenuation factor 
            for i = 1 : obj.horizon - 1
                simdata.prey(:, 1 + obj.params.n * i : obj.params.n * (i + 1)) = simdata.prey(:, 1 + obj.params.n * i : obj.params.n * (i + 1)) * (beta^(i - 1));
                real(:, 1 + obj.params.n * i : obj.params.n * (i + 1)) = real(:, 1 + obj.params.n * i : obj.params.n * (i + 1)) * (beta^(i - 1));
            end
            preydata = zeros(size(real, 1) * (obj.horizon - 1), obj.params.n);
            realdata = zeros(size(real, 1) * (obj.horizon - 1), obj.params.n);
            if obj.horizon > 1
                for i = 1 : obj.params.n
                    preytem = simdata.prey(:,i + obj.params.n : obj.params.n: size(simdata.prey, 2));
                    preydata(:,i) = preytem(:);
                    realtem = real(:,i + obj.params.n : obj.params.n: size(simdata.prey, 2));
                    realdata(:,i) = realtem(:);
                end
                datalength = size(real, 1);
            end
        end

        % model validation
        function [results, loss] = val(obj)
            mod = obj.model;
            l = 0;
            results = cell(size(obj.valdata)); % store modeling results
            for i = 1 : length(obj.valdata)
                results{i} = obj.val_model(mod, obj.valdata{i});
                [datalength, prey, real] = obj.get_error(results{i}.sim, results{i}.real);
                if i == 1
                    preydata = prey;
                    realdata = real;
                else
                    preydata = [preydata; prey];
                    realdata = [realdata; real];
                end
                l = l + datalength;
            end
            loss = sum(sqrt(sum((preydata - realdata).^2) / l) ./ (max(realdata) - min(realdata))); % normalize
        end
    end
end
