/* Copyright 2015 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.samsungxr.asynchronous;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import com.samsungxr.SXRAtlasInformation;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Async resource loading: Atlas texture information.
 *
 * Parse the JSON file containing the atlas information for
 * each mesh mapped at the atlas texture.
 */
abstract class AsyncAtlasInfo {

    /**
     * This method is a sync parse to the JSON stream of atlas information.
     *
     * @return List of atlas information.
     */
    public static List<SXRAtlasInformation> loadAtlasInformation(InputStream ins) {
        try {
            int size = ins.available();
            byte[] buffer = new byte[size];

            ins.read(buffer);

            return loadAtlasInformation(new JSONArray(new String(buffer, "UTF-8")));
        } catch (JSONException je) {
            je.printStackTrace();
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        return null;
    }

    private static List<SXRAtlasInformation> loadAtlasInformation(JSONArray jsonInfo) throws JSONException {
        List<SXRAtlasInformation> list = new ArrayList<SXRAtlasInformation>();
        int length = jsonInfo.length();

        for (int i = 0; i < length; i++) {
            if (jsonInfo.isNull(i))
                continue;
            list.add(parseAtlasInformation(jsonInfo.getJSONObject(i)));
        }

        return list;
    }

    private static SXRAtlasInformation parseAtlasInformation(JSONObject jsonObj) throws JSONException {
        String name = jsonObj.getString("name");
        float[] offset = {(float) jsonObj.getDouble("offset.x"), (float) jsonObj.getDouble("offset.y")};
        float[] scale = {(float) jsonObj.getDouble("scale.x"), (float) jsonObj.getDouble("scale.y")};

        return new SXRAtlasInformation(name, offset, scale);
    }

}
