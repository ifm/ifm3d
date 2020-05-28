#
# Copyright (C) 2019 ifm electronic, gmbh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import pytest
import time
import json

import ifm3dpy

@pytest.fixture(scope="module")
def cam():
    cam = ifm3dpy.Camera()
    yield cam

def test_factorydefaults(cam):
    cam.factory_reset()
    time.sleep(6)
    cam.device_type()

def test_defaultcredentials(cam):
    assert cam.ip == ifm3dpy.DEFAULT_IP
    assert cam.xmlrpc_port == ifm3dpy.DEFAULT_XMLRPC_PORT
    assert cam.password == ifm3dpy.DEFAULT_PASSWORD

def test_applicationlist(cam):
    app_list = cam.application_list()
    assert len(app_list) == 1 # Factory defaults, we can assume this.
    assert app_list[0]['Active']

def test_sessionmanagement(cam):
    sid = cam.request_session()
    assert cam.session_id == sid
    retval = cam.cancel_session()
    assert retval

    # Ensure the session is closed when the object goes out of scope
    cam2 = ifm3dpy.Camera()
    sid = cam2.request_session()
    del cam2
    sid = cam.request_session()
    assert cam.session_id == sid
    assert cam.cancel_session()
    assert "" == cam.session_id

    cam2 = ifm3dpy.Camera()
    sid = cam2.request_session()
    ## OK, pretend `cam2` crashed ... we want to create a new session
    ## but we will get an exception
    with pytest.raises(RuntimeError):
        cam.request_session()
    ## Let's now use cam to cancel the sid from the other camera
    assert cam.cancel_session(sid)
    ## Now make a new session and cancel it
    sid = cam.request_session()
    assert cam.cancel_session()

    ## Cam2's dtor will try to close the session as well, which fails with
    ## an error in the log. This error can be ignored.
    del cam2

def test_copydeleteapplication(cam):
    if cam.is_O3X():
        return

    app_list = cam.application_list()
    assert len(app_list) == 1
    idx = cam.copy_application(1)
    app_list = cam.application_list()
    assert len(app_list) == 2

    cam.delete_application(idx)
    app_list = cam.application_list()
    assert len(app_list) == 1

def test_copydeleteexceptions(cam):
    with pytest.raises(RuntimeError):
        cam.copy_application(-1)
    with pytest.raises(RuntimeError):
        cam.delete_application(-1)

def test_createdeleteapplication(cam):
    if cam.is_O3X():
        return

    app_list = cam.application_list()
    assert len(app_list) == 1

    idx = -1
    app_types = cam.application_types()
    for s in app_types:
        idx = cam.create_application(s)
        app_list = cam.application_list()
        assert len(app_list) == 2
        cam.delete_application(idx)
        app_list = cam.application_list()
        assert len(app_list) == 1

def test_createapplicationexception(cam):
    with pytest.raises(RuntimeError):
        cam.create_application('Foo')

def test_importexportapplication(cam):
    app_list = cam.application_list()
    assert len(app_list) == 1
    idx = app_list[0]['Index']

    if cam.is_O3X():
        exp_bytes = cam.export_ifm_app(idx)
        new_idx = cam.import_ifm_app(exp_bytes)
        assert idx == new_idx # single app on O3X
    else:
        exp_bytes = cam.export_ifm_app(idx)
        new_idx = cam.import_ifm_app(exp_bytes)

        app_list = cam.application_list()
        assert len(app_list) == 2

        cam.delete_application(new_idx)
        app_list = cam.application_list()
        assert len(app_list) == 1

def test_importexportconfig(cam):
    exp_bytes = cam.export_ifm_config()
    cam.import_ifm_config(exp_bytes, cam.import_flags.GLOBAL)

def test_activeapplication(cam):
    app_list = cam.application_list()
    assert len(app_list) == 1

    # The rest of the test is invalid for O3X
    if cam.is_O3X():
        return

    # Create a new application using JSON syntax
    cam.from_json(json.loads('{"Apps":[{}]}'))
    app_list = cam.application_list()
    assert len(app_list) == 2

    # We expect the new application to be at index 2
    idx = -1
    for a in app_list:
        if not a["Active"]:
            idx = a["Index"]
            break
    assert idx == 2

    # Mark the application at index 2 as active
    cam.from_json(json.loads('{"Device":{"ActiveApplication":"2"}}'))
    app_list = cam.application_list()
    idx = -1
    for a in app_list:
        if a["Active"]:
            idx = a["Index"]
            break
    assert idx == 2

    # Delete the application at index 2
    cam.delete_application(idx)

    # Should only have one applicaiton now
    app_list = cam.application_list()
    assert len(app_list) == 1

    # The only application on the camera will not be active
    idx = -1
    for a in app_list:
        if a["Active"]:
            idx = a["Index"]
            break
    assert idx == -1

    # Mark application 1 as active
    cam.from_json(json.loads('{"Device":{"ActiveApplication":"1"}}'))
    app_list = cam.application_list()
    idx = -1
    for a in app_list:
        if a["Active"]:
            idx = a["Index"]
            break
    assert idx == 1

def test_imagertypes(cam):
    dump = cam.to_json()
    curr_im_type = dump['ifm3d']['Apps'][0]['Imager']['Type']
    im_types = cam.imager_types()
    j = json.loads('{"Apps":[]}')
    for it in im_types:
        print('Testing: ' + it)
        dump['ifm3d']['Apps'][0]['Imager']['Type'] = it
        j['Apps'] = dump['ifm3d']['Apps']
        cam.from_json(j)
        dump = cam.to_json()
        assert dump['ifm3d']['Apps'][0]['Imager']['Type'] == it

    dump['ifm3d']['Apps'][0]['Imager']['Type'] = curr_im_type
    j['Apps'] = dump['ifm3d']['Apps']
    cam.from_json(j)

def test_filters(cam):
    app_list = cam.application_list()
    assert len(app_list) == 1

    # Create a median spatial filter
    j = json.loads(
        '{"Apps":[{"Index":"1", "Imager": {"SpatialFilterType":"1"}}]}')
    cam.from_json(j)

    # a-priori we know the median filter has a MaxSize param,
    # by default it is 3x3
    dump = cam.to_json()

    mask_size = int(cam.mfilt_mask_size._3x3)
    if not cam.is_O3X():
        mask_size = \
            int(dump['ifm3d']['Apps'][0]['Imager']\
                ['SpatialFilter']['MaskSize'])

    assert mask_size == int(cam.mfilt_mask_size._3x3)

    # get rid of the spatial filter
    j = json.loads(
        '{"Apps":[{"Index":"1", "Imager": {"SpatialFilterType":"0"}}]}')
    cam.from_json(j)

    # Create a mean temporal filter
    j = json.loads(
        '{"Apps":[{"Index":"1", "Imager": {"TemporalFilterType":"1"}}]}')
    cam.from_json(j)

    # a-priori we know the mean filter averages 2 images by default
    dump = cam.to_json()

    n_imgs = 2
    if not cam.is_O3X():
        n_imgs = \
            int(dump['ifm3d']['Apps'][0]['Imager']\
                    ['TemporalFilter']['NumberOfImages'])

    assert n_imgs == 2

    # get rid of the temporal filter
    j = json.loads(
        '{"Apps":[{"Index":"1", "Imager": {"TemporalFilterType":"0"}}]}')
    cam.from_json(j)

def test_json(cam):
    cam.from_json(cam.to_json())
    j = json.loads('{"Device":{"Name":"ifm3d unit test"}}')
    cam.from_json(j)

def test_time(cam):
    dump = cam.to_json()
    if not dump['ifm3d']['Time']:
        return

    # 1. Check mutating the "WaitSyncTries" parameters
    n_tries = dump['ifm3d']['Time']['WaitSyncTries']
    n_tries_new = "2" if n_tries == "1" else "1"
    dump['ifm3d']['Time']['WaitSyncTries'] = n_tries_new

    cam.from_json(dump)
    dump = cam.to_json()
    n_tries = dump['ifm3d']['Time']['WaitSyncTries']
    assert n_tries == n_tries_new

    # 2. Make usre CurrentTime is treated as read-only
    curr_time = int(dump['ifm3d']['Time']['CurrentTime'])
    dump['ifm3d']['Time']['CurrentTime'] = "0"
    cam.from_json(dump)
    # Make sure at least one full second passes
    time.sleep(1)
    dump = cam.to_json()
    curr_time_new = int(dump['ifm3d']['Time']['CurrentTime'])
    assert curr_time_new > curr_time

    # 3. Test setting the NTP servers && activating NTP
    j = json.loads('{"Time":{"NTPServers":"91.189.89.198",' +
                   '"SynchronizationActivated":"true"}}')
    cam.from_json(j)
    dump = cam.to_json()
    ip = dump['ifm3d']['Time']['NTPServers']
    active = dump['ifm3d']['Time']['SynchronizationActivated']
    assert "91.189.89.198" == ip
    assert "true" == active

    # 4. Now, undo what we did above
    j = json.loads('{"Time":{"NTPServers":"",' +
                   '"SynchronizationActivated":"false"}}')
    cam.from_json(j)
    dump = cam.to_json()
    ip = dump['ifm3d']['Time']['NTPServers']
    active = dump['ifm3d']['Time']['SynchronizationActivated']
    assert "" == ip
    assert "false" == active

    # 5. Set the time to "now"
    cam.set_current_time()

def test_temporaryparameters(cam):
    params = {"imager_001/ExposureTime" : "6000"}
    sid = cam.request_session()
    cam.set_temporary_application_parameters(params)

    params = {"imager_001/ExposureTime" : "5000",
              "imager_001/ExposureTimeRatio" : "40"}
    cam.set_temporary_application_parameters(params)
    cam.cancel_session(sid)
