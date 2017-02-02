%%
%% Copyright (C) 2010 by Karinne Ramirez-Amaro
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
/* ***************************************
	Author:	Karinne Ramirez-Amaro
	E-mail:	karinne.ramirez@tum.de

 This library contains predicates used for the 
 inference method using prolog queries. 


 NOTE: The following symbols are used to describe the parameter 
of the predicates
 + The argument is instantiated at entry.
 - The argument is not instantiated at entry.
 ? The argument is unbound or instantiated at entry.

*/



:- module(myReasoning_utils,
    [
	comp_onTopTable/2,
	create_instance_from_class/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_objects')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
% :- rdf_db:rdf_register_ns(semRoom_semantic_map, 'http://knowrob.org/kb/semRoom_semantic_map.owl#', [keep(true)]).



%%%%%%%%%%%%%% Costum computables %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Property computables %%%%%%%%%%%%%%%%%%%%%%%%


%% Compare the Z position of the objects to determine if they are on-top of the table
%%% comp_onTopTable(?Obj, ?ValueNewObj) is nondet.
%
% Find all Objects that are onTop of the table and retreive their Z positions
%
% @param Obj     Object instance of interest
% @param ValueNewObj    Z position of the detected object
% 
%
comp_onTopTable(Obj, ValueNewObj):-
	owl_has(A, knowrob:objectActedOn, Obj), 
	owl_has(A, knowrob:eventOccursAt, Robj),
	owl_has(Robj, knowrob:m23, ValueObj), 
	strip_literal_type(ValueObj, ValueNewObj),
	atom_to_term(ValueNewObj, TermObjZ, _),

 	owl_individual_of(Table,  knowrob:'KitchenTable'),
	owl_has(Atable, knowrob:objectActedOn, Table),
	owl_has(Atable, knowrob:eventOccursAt, Vtable),
	owl_has(Vtable, knowrob:m23, ValueTable), 
	strip_literal_type(ValueTable, ValueNewTable),
	atom_to_term(ValueNewTable, TermTableZ, _),
	
    	TermObjZ>TermTableZ,

	write('Found an object on top of the table: '), write(Obj), nl.



% This function will create an instance of a desired class
% create_instance_from_class(+Class, +Instance_ID, ?Instance)
% The created instance will have the predicate/property rdf:type 
% to correctly inheritate the properties of its super-classes
%
% @param Class		represents the name of the class where the instance will be created. 
%			Class could be of two forms: 
%			Class='http://knowrob.org/kb/knowrob.owl#Cup' (the desired URL)
%			Class='Cup'  <- if this is chosen then it will be created in Knowrob ontology
% @param Instance_ID	is the new ID that the instance will have
% @param Instance	asserted new instance

create_instance_from_class(Class, Instance_ID, Instance) :-
	% We need to find out if the Class has URL or not
	((concat_atom(List, '#', Class),length(List,Length),Length>1) ->
	( % Class has already a URI
	   Class_path=Class );
	  % When the class does not have URL, will get the knowrob path
	(atom_concat('http://knowrob.org/kb/knowrob.owl#', Class, Class_path),
	write(Class_path), nl
 	)),
	% Create the path of the new instance
	atom_concat(Class_path,  '_', Class2),
	atom_concat(Class2, Instance_ID, Instance),
	
	% assert/create the new instance
	rdf_assert(Instance, rdf:type, Class_path).


