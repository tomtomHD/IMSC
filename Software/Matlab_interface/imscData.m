classdef imscData
    %IMSCDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Measured Impedances:
        impedances = struct('fromElectrode',{}, 'toElectrode',{}, 'measureElectrode',{}, 'realValue',{}, 'imagValue',{}, 'magnitude',{});
        % Measured Voltages:
        voltages   = struct('fromElectrode',{}, 'toElectrode',{}, 'measureElectrode',{}, 'realValue',{}, 'imagValue',{}, 'magnitude',{});
        % Measured other Data (temporary):
        other      = struct('fromElectrode',{}, 'toElectrode',{}, 'measureElectrode',{}, 'realValue',{}, 'imagValue',{}, 'magnitude',{});
        % Measured EIT-Data:
        eit        = struct('fromElectrode',{}, 'toElectrode',{}, 'measureElectrode',{}, 'realValue',{}, 'imagValue',{}, 'magnitude',{});
    end
    
    methods
        function obj = imscData()
            %IMSCDATA Construct an instance of this class
        end
    end
end

