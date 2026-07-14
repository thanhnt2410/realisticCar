function fis = createFuzzyRules()

NB = 1;
NM = 2;
NS = 3;
ZO = 4;
PS = 5;
PM = 6;
PB = 7;

fis = sugfis("Name","FuzzyPID_Sugeno");

%% Inputs
fis = addInput(fis,[-3 3],"Name","Error");
fis = addInput(fis,[-3 3],"Name","dError");

%% Outputs
fis = addOutput(fis,[-3 3],"Name","dKp");
fis = addOutput(fis,[-3 3],"Name","dKi");
fis = addOutput(fis,[-3 3],"Name","dKd");

%% Membership Functions
fis = addSevenMF(fis,"Error");
fis = addSevenMF(fis,"dError");
fis = addSevenConstMF(fis,"dKp");   % output đổi sang constant
fis = addSevenConstMF(fis,"dKi");
fis = addSevenConstMF(fis,"dKd");

%% ================= Rule Table =================

ruleKp = [
PB PB PM PM PS ZO ZO; % E(t) = NB
PB PB PM PS PS ZO NS; % E(t) = NM
PM PM PM PS ZO NS NS; % E(t) = NS
PM PM PS ZO NS NM NM; % E(t) = ZO
PS PS ZO NS NS NM NM; % E(t) = PS
PS ZO NS NM NM NM NB; % E(t) = PM
ZO ZO NM NM NM NB NB; % E(t) = PB
];

% ruleKi = [
% NB NB NM NM NS ZO ZO; % E(t) = NB
% NB NB NM NS NS ZO ZO; % E(t) = NM
% NB NM NS NS ZO PS PS; % E(t) = NS
% NM NM NS ZO PS PM PM; % E(t) = ZO
% NM NS ZO PS PS PM PB; % E(t) = PS
% ZO ZO PS PS PM PB PB; % E(t) = PM
% ZO ZO PS PM PM PB PB; % E(t) = PB
% ];
ruleKi = [
NS NS NS NS ZO ZO ZO
NS NS NS ZO ZO PS PS
NS NS ZO ZO PS PS PS
NS ZO ZO ZO PS PS PM
ZO ZO PS PS PS PM PM
ZO PS PS PM PM PM PB
ZO PS PM PM PB PB PB
];

ruleKd = [
PS NS NB NB NB NM PS; % E(t) = NB
PS NS NB NM NM NS ZO; % E(t) = NM
ZO NS NM NM NS NS ZO; % E(t) = NS
ZO NS NS NS NS NS ZO; % E(t) = ZO
ZO ZO ZO ZO ZO ZO ZO; % E(t) = PS
PB NS PS PS PS PS PB; % E(t) = PM
PB PM PM PM PS PS PM; % E(t) = PB
];

%% Convert Rule Table

rules = zeros(49,7);

idx = 1;

for i = 1:7
    for j = 1:7

        rules(idx,:) = [ ...
            i ...
            j ...
            ruleKp(i,j) ...
            ruleKi(i,j) ...
            ruleKd(i,j) ...
            1 ...
            1];

        idx = idx + 1;

    end
end

fis = addRule(fis,rules);

end


function fis = addSevenMF(fis,varName)

fis = addMF(fis,varName,"gaussmf",[0.6 -3],"Name","NB");
fis = addMF(fis,varName,"trimf",[-3 -2 -1],"Name","NM");
fis = addMF(fis,varName,"trimf",[-2 -1 0],"Name","NS");
fis = addMF(fis,varName,"trimf",[-1 0 1],"Name","ZO");
fis = addMF(fis,varName,"trimf",[0 1 2],"Name","PS");
fis = addMF(fis,varName,"trimf",[1 2 3],"Name","PM");
fis = addMF(fis,varName,"gaussmf",[0.6 3],"Name","PB");
end

function fis = addSevenConstMF(fis,varName)
% Dùng cho OUTPUT (dKp, dKi, dKd) - Sugeno zero-order (hằng số)
fis = addMF(fis,varName,"constant",-3,"Name","NB");
fis = addMF(fis,varName,"constant",-2,"Name","NM");
fis = addMF(fis,varName,"constant",-1,"Name","NS");
fis = addMF(fis,varName,"constant", 0,"Name","ZO");
fis = addMF(fis,varName,"constant", 1,"Name","PS");
fis = addMF(fis,varName,"constant", 2,"Name","PM");
fis = addMF(fis,varName,"constant", 3,"Name","PB");
end