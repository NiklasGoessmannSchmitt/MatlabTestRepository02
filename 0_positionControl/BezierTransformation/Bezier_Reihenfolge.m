function [bezier_sortiert] = Bezier_Reihenfolge(bezier_unsortiert,size)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


Startpunkt{1,1}=  bezier_unsortiert{1,1};                   %Eigentlich der Startpunkt, wird spaeter weiterverwendert als Zwischenpunkt
Zwischenpunkt{1,1}= Startpunkt{1,1};
bezier_sortiert=[];                                         %cell(1,size);            %Initalisierung des Feldes
bezier_sortiert{1,1}= bezier_unsortiert{1,1};

f=1;

for k=1:size
    Hilfsfeld={};                   %Speicher bei mehreren moeglichen Wegen bzw. Abzeigen
    noFound=0;
    speicher={};

    %if ~isemptY(bezier_unsortiert{1,k})

    for i=2:size            % Abgleich des Zwischenpunktes mit allen Punkten


        befuellt = isempty(bezier_unsortiert{1,i}.x);                           %Überprüfen ob der Eintrag Werte gespeichert hat
        if befuellt == 0                                                    %Leere Arrays und damit breits benutzte werden nicht weiter betrachten

            l_z=length(Zwischenpunkt{1,1}.x);
            l_p=length(bezier_unsortiert{1,i}.x);
            %Absolutwerte der Differenzen der Punkte berechnen
            absolut_1_Anfangspunkt= abs(bezier_unsortiert{1,i}.x(1)- Zwischenpunkt{1,1}.x(l_z));
            absolut_2_Anfangspunkt= abs(bezier_unsortiert{1,i}.y(1)- Zwischenpunkt{1,1}.y(l_z));

            absolut_1_Endpunkt= abs(bezier_unsortiert{1,i}.x(l_p) - Zwischenpunkt{1,1}.x(l_z));
            absolut_2_Endpunkt= abs(bezier_unsortiert{1,i}.y(l_p) - Zwischenpunkt{1,1}.y(l_z));

            if  (absolut_1_Anfangspunkt <= 0.5) && (absolut_2_Anfangspunkt < 0.5) || (absolut_1_Endpunkt <= 0.5) && (absolut_2_Endpunkt < 0.5)

                % Unterscheidung und dann Punkte richtig speciehern
                if (absolut_1_Anfangspunkt <= 0.1) && (absolut_2_Anfangspunkt < 0.1)
                    Hilfsfeld{1,noFound+1}.x=bezier_unsortiert{1,i}.x;
                    Hilfsfeld{1,noFound+1}.y=bezier_unsortiert{1,i}.y;
                    zaehler= noFound+1;
                    speicher{zaehler}=i;
                    noFound=noFound+1;

                elseif (absolut_1_Endpunkt <= 0.1) && (absolut_2_Endpunkt < 0.1)
                    if length(bezier_unsortiert{1,i}.x) == 3
                        Hilfsfeld{1,noFound+1}.x(1)=bezier_unsortiert{1,i}.x(l_p);
                        Hilfsfeld{1,noFound+1}.y(1)=bezier_unsortiert{1,i}.y(l_p);
                        Hilfsfeld{1,noFound+1}.x(2)=bezier_unsortiert{1,i}.x(2);
                        Hilfsfeld{1,noFound+1}.y(2)=bezier_unsortiert{1,i}.y(2);
                        Hilfsfeld{1,noFound+1}.x(3)=bezier_unsortiert{1,i}.x(1);
                        Hilfsfeld{1,noFound+1}.y(3)=bezier_unsortiert{1,i}.y(1);
                        zaehler= noFound+1;
                        speicher{zaehler}=i;
                        noFound=noFound+1;

                    elseif length(bezier_unsortiert{1,i}.x) == 4
                        Hilfsfeld{1,noFound+1}.x(1)=bezier_unsortiert{1,i}.x(4);
                        Hilfsfeld{1,noFound+1}.y(1)=bezier_unsortiert{1,i}.y(4);
                        Hilfsfeld{1,noFound+1}.x(2)=bezier_unsortiert{1,i}.x(3);
                        Hilfsfeld{1,noFound+1}.y(2)=bezier_unsortiert{1,i}.y(3);
                        Hilfsfeld{1,noFound+1}.x(3)=bezier_unsortiert{1,i}.x(2);
                        Hilfsfeld{1,noFound+1}.y(3)=bezier_unsortiert{1,i}.y(2);
                        Hilfsfeld{1,noFound+1}.x(4)=bezier_unsortiert{1,i}.x(1);
                        Hilfsfeld{1,noFound+1}.y(4)=bezier_unsortiert{1,i}.y(1);
                        zaehler= noFound+1;
                        speicher{zaehler}=i;
                        noFound=noFound+1;
                    end
                end
            end
            end
        end



    % Ueberpruefung der moeglichen Abzweige
    if noFound==1
        bezier_sortiert{1,k+1}.x=Hilfsfeld{1,1}.x;
        bezier_sortiert{1,k+1}.y=Hilfsfeld{1,1}.y;
        Zwischenpunkt{1,1}.x=[];
        Zwischenpunkt{1,1}.y=[];
        Zwischenpunkt{1,1}= bezier_sortiert{1,k+1};
        eintrag= speicher{noFound};
        bezier_unsortiert{1,eintrag}.x=[];
        bezier_unsortiert{1,eintrag}.y=[];


    elseif noFound>1
        str=['Welcher Abzweig soll gewählt werden? Es gibt ' , num2str(noFound)];
        prompt =  str;
        abzweig = input(prompt);
        bezier_sortiert{1,k+1}.x=Hilfsfeld{abzweig}.x;
        bezier_sortiert{1,k+1}.y=Hilfsfeld{abzweig}.y;
        Zwischenpunkt{1,1}= bezier_sortiert{1,k+1};
        eintrag= speicher{abzweig};
        bezier_unsortiert{1,eintrag}.x=[];
        bezier_unsortiert{1,eintrag}.y=[];


    elseif noFound==0
        anzahl = num2str(f);
        %disp('Keine Wert gefunden');
        disp(anzahl);
        f=f+1;

    end

end



end