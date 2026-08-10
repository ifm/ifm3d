{{ fullname | escape | underline}}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}
   :members:
   :show-inheritance:
   :inherited-members: ndarray, int, Enum

   {% block methods %}
   {% if '__init__' in members and documented_members(fullname, ['__init__']) %}
   .. automethod:: __init__
   {% endif %}

   {% set own_methods = documented_members(fullname, methods) %}
   {% if own_methods %}
   .. rubric:: {{ _('Methods') }}

   .. autosummary::
   {% for item in own_methods %}
      ~{{ name }}.{{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block nested %}
   {#- Nested types (the enums pybind11 exposes inside the device and client
       classes) appear in ``members`` but in neither ``methods`` nor
       ``attributes``, so they would otherwise be absent from the summary. -#}
   {% set nested = [] %}
   {% for item in members %}
   {% if item not in methods and item not in attributes and item not in inherited_members and not item.startswith('_') %}
   {% set _ = nested.append(item) %}
   {% endif %}
   {% endfor %}
   {% set own_nested = documented_members(fullname, nested) %}
   {% if own_nested %}
   .. rubric:: {{ _('Nested types') }}

   .. autosummary::
   {% for item in own_nested %}
      ~{{ name }}.{{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block attributes %}
   {% set own_attributes = documented_members(fullname, attributes) %}
   {% if own_attributes %}
   .. rubric:: {{ _('Attributes') }}

   .. autosummary::
   {% for item in own_attributes %}
      ~{{ name }}.{{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}
